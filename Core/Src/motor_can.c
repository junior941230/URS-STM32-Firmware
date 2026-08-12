#include "motor_can.h"

#include "main.h"

#include <string.h>

/*
 * Motor CAN 模組架構
 * ------------------
 * 1. USB command 層呼叫 MotorCAN_Start*() 啟動一項非同步操作。
 * 2. FDCAN ISR 只把收到的 frame 放入 motor_rx_queue，不在中斷內跑 state machine。
 * 3. 主迴圈反覆呼叫 MotorCAN_Process()，依 frame、timeout、EMS 與 bus error 推進狀態。
 * 4. 完成或失敗結果放入 motor_event_queue，再由 USB command 層轉成文字回覆。
 *
 * 這種分工讓 ISR 保持短小，也避免在 callback 內呼叫可能等待硬體的 HAL API。
 */

/* Queue 容量與協定輸入範圍。ring buffer 會保留一格來區分 full 與 empty。 */
#define MOTOR_CAN_BUS_COUNT             2U
#define MOTOR_CAN_MAX_STANDARD_ID       0x7FFU
#define MOTOR_CAN_RX_QUEUE_SIZE         16U
#define MOTOR_CAN_EVENT_QUEUE_SIZE      8U

/* 各 state 等待馬達回覆的上限；所有 deadline 都以 HAL_GetTick() 的 ms 為單位。 */
#define MOTOR_CAN_PROBE_TIMEOUT_MS      250U
#define MOTOR_CAN_COMMAND_TIMEOUT_MS    800U
#define MOTOR_CAN_VERIFY_DELAY_MS       200U
#define MOTOR_CAN_RESTART_DELAY_MS      1000U

/* TEST 使用固定且保守的短行程參數，避免測試命令造成長時間移動。 */
#define MOTOR_CAN_TEST_TIMEOUT_MS       2000U
#define MOTOR_CAN_TEST_SPEED_RPM        30U
#define MOTOR_CAN_TEST_ACCELERATION     2U
#define MOTOR_CAN_TEST_RUNTIME_10MS     50U

/* HOME 的 torque 固定為 300 mA；使用者可設定速度、offset 與 timeout。 */
#define MOTOR_CAN_HOME_TORQUE_MA        300U
#define MOTOR_CAN_HOME_MIN_TIMEOUT_MS   1000U
#define MOTOR_CAN_HOME_MAX_TIMEOUT_MS   120000U
#define MOTOR_CAN_HOME_TIMEOUT_GUARD_MS 1000U

/* STM32 端支援的兩種 CAN nominal bitrate，以及 170 MHz kernel clock 對應 prescaler。 */
#define MOTOR_CAN_RATE_500_KBPS         500U
#define MOTOR_CAN_RATE_1M_KBPS          1000U
#define MOTOR_CAN_PRESCALER_500K        20U
#define MOTOR_CAN_PRESCALER_1M          10U

/* Bus-off/error-passive 復原在主迴圈重試，避免失敗時卡住整個 super-loop。 */
#define MOTOR_CAN_RECOVERY_RETRY_MS      250U
#define MOTOR_CAN_NOTIFICATIONS         (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                                         FDCAN_IT_BUS_OFF | \
                                         FDCAN_IT_ERROR_PASSIVE | \
                                         FDCAN_IT_ERROR_WARNING)
#define MOTOR_CAN_RECOVERY_ERROR_FLAGS  (FDCAN_IT_BUS_OFF | \
                                         FDCAN_IT_ERROR_PASSIVE)

typedef struct
{
  /* bus 使用對外的 1-based 編號；id 是 11-bit Standard CAN identifier。 */
  uint8_t bus;
  uint16_t id;
  /* length 包含最後一個 checksum byte。 */
  uint8_t length;
  uint8_t data[8];
} MotorCAN_RxFrame;

/*
 * 單一操作 state machine。
 *
 * state 名稱描述「目前正在等待哪一個回覆」。某些 timeout 是正常流程的一部分，
 * 例如 SET_PROBE_NEW timeout 代表新 ID 尚未被占用，此時才真正送出改 ID 命令。
 */
typedef enum
{
  MOTOR_STATE_IDLE = 0,

  /* INFO：等待 0x40 version reply。 */
  MOTOR_STATE_INFO_WAIT,

  /* SET_ID：確認舊 ID 存在、新 ID 未占用，再修改、儲存並驗證。 */
  MOTOR_STATE_SET_PROBE_OLD,
  MOTOR_STATE_SET_PROBE_NEW,
  MOTOR_STATE_SET_WAIT_ACK,
  MOTOR_STATE_SET_FIND_NEW,
  MOTOR_STATE_SET_FIND_OLD,
  MOTOR_STATE_SET_WAIT_SAVE,
  MOTOR_STATE_SET_WAIT_VERIFY_DELAY,
  MOTOR_STATE_SET_WAIT_VERIFY,

  /* TEST：確認馬達存在、Enable、執行固定 30 RPM / 500 ms 測試。 */
  MOTOR_STATE_TEST_PROBE,
  MOTOR_STATE_TEST_WAIT_ENABLE,
  MOTOR_STATE_TEST_WAIT_RUN,

  /* HOME：逐項寫入 homing 設定、儲存、Enable，最後等待 homing 結果。 */
  MOTOR_STATE_HOME_PROBE,
  MOTOR_STATE_HOME_WAIT_MODE,
  MOTOR_STATE_HOME_WAIT_SWITCH_LEVEL,
  MOTOR_STATE_HOME_WAIT_PARAMETERS,
  MOTOR_STATE_HOME_WAIT_OFFSET,
  MOTOR_STATE_HOME_WAIT_TRIGGER,
  MOTOR_STATE_HOME_WAIT_SAVE,
  MOTOR_STATE_HOME_WAIT_ENABLE,
  MOTOR_STATE_HOME_WAIT_EXECUTE,

  /* PROVISION_1M：先在 500K 找馬達，再切換、儲存、重啟並於 1M 驗證。 */
  MOTOR_STATE_RATE_PROBE_500,
  MOTOR_STATE_RATE_WAIT_SET_ACK_500,
  MOTOR_STATE_RATE_WAIT_SAVE_500,
  MOTOR_STATE_RATE_WAIT_RESET_500,
  MOTOR_STATE_RATE_FIND_1M,
  MOTOR_STATE_RATE_WAIT_SAVE_1M,
  MOTOR_STATE_RATE_WAIT_RESET_1M,
  MOTOR_STATE_RATE_WAIT_REBOOT_1M,
  MOTOR_STATE_RATE_WAIT_VERIFY
} MotorCAN_State;

typedef struct
{
  /* operation 是對外操作名稱；state 是該操作內部目前所處步驟。 */
  MotorCAN_Operation operation;
  MotorCAN_State state;

  /* 所有操作共用的目標；SET_ID 另外追蹤 old/new/active ID。 */
  uint8_t bus;
  uint16_t id;
  uint16_t old_id;
  uint16_t new_id;
  uint16_t active_id;

  /* ACK 可能因馬達立即切換 ID/bitrate 而遺失，旗標用來支援後續實際探測。 */
  uint8_t set_ack_received;
  uint8_t rate_save_ack_received;

  /* HOME 的使用者參數會跨越多個 request/reply state，因此集中保存在 context。 */
  uint8_t home_direction;
  uint16_t home_high_speed_rpm;
  uint16_t home_low_speed_rpm;
  int32_t home_origin_offset;
  uint32_t home_timeout_ms;

  /* 絕對 tick deadline；用 signed subtraction 比較以安全跨越 uint32_t overflow。 */
  uint32_t deadline;
} MotorCAN_Context;

/* bus 1 對應 handles[0]，bus 2 對應 handles[1]。online=0 時禁止送 frame。 */
static FDCAN_HandleTypeDef *motor_can_handles[MOTOR_CAN_BUS_COUNT];
static uint16_t motor_bus_bitrate_kbps[MOTOR_CAN_BUS_COUNT];
static uint8_t motor_bus_online[MOTOR_CAN_BUS_COUNT];
static MotorCAN_Context motor_context;
static uint8_t motor_can_ready;

/*
 * RX queue 是 single-producer/single-consumer ring buffer：
 * FDCAN ISR 寫入 head，主迴圈讀取 tail。共享索引使用 volatile。
 */
static MotorCAN_RxFrame motor_rx_queue[MOTOR_CAN_RX_QUEUE_SIZE];
static volatile uint8_t motor_rx_head;
static volatile uint8_t motor_rx_tail;
static volatile uint8_t motor_rx_overflow;
static volatile uint32_t motor_bus_error_flags[MOTOR_CAN_BUS_COUNT];
static uint8_t motor_bus_recovery_pending[MOTOR_CAN_BUS_COUNT];
static uint32_t motor_bus_recovery_deadline[MOTOR_CAN_BUS_COUNT];

/*
 * Event queue 只在主迴圈 context 存取。MotorCAN 寫入 head，USB command 讀取 tail；
 * queue 滿時保留最新事件並捨棄最舊事件，避免新錯誤被舊訊息遮住。
 */
static MotorCAN_Event motor_event_queue[MOTOR_CAN_EVENT_QUEUE_SIZE];
static uint8_t motor_event_head;
static uint8_t motor_event_tail;

/**
  * @brief 驗證對外使用的 CAN bus 編號。
  * @param bus 1-based bus 編號，目前有效值為 1 或 2。
  * @retval 1 表示有效；0 表示超出範圍。
  */
static uint8_t MotorCAN_IsValidBus(uint8_t bus)
{
  return (bus >= 1U) && (bus <= MOTOR_CAN_BUS_COUNT);
}

/**
  * @brief 驗證可作為單一馬達目標的 Standard CAN ID。
  * @param id 11-bit CAN identifier。
  * @retval 1 表示介於 1..0x7FF；0 表示為 broadcast ID 0 或超出範圍。
  * @note 本模組禁止對 broadcast ID 執行改設定、測試與 homing。
  */
static uint8_t MotorCAN_IsValidNodeId(uint16_t id)
{
  /* ID 0 是 broadcast；改設定與測試一律禁止使用。 */
  return (id >= 1U) && (id <= MOTOR_CAN_MAX_STANDARD_ID);
}

/**
  * @brief 判斷指定的絕對 tick deadline 是否已到期。
  * @param now 目前的 HAL tick，單位為 ms。
  * @param deadline 先前儲存的絕對 HAL tick deadline。
  * @retval 1 表示已到期；0 表示仍在等待時間內。
  * @note 使用 signed delta，比較在 uint32_t tick overflow 前後仍然有效。
  */
static uint8_t MotorCAN_DeadlineReached(uint32_t now, uint32_t deadline)
{
  /* signed delta 寫法在 tick overflow 前後都能正確比較相差小於 2^31 ms 的時間。 */
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

/**
  * @brief 檢查 STM32 端是否支援指定的 CAN nominal bitrate。
  * @param bitrate_kbps 以 kbit/s 表示的速率。
  * @retval 1 表示為 500 或 1000 kbit/s；0 表示不支援。
  */
static uint8_t MotorCAN_IsSupportedBitrate(uint16_t bitrate_kbps)
{
  return (bitrate_kbps == MOTOR_CAN_RATE_500_KBPS) ||
         (bitrate_kbps == MOTOR_CAN_RATE_1M_KBPS);
}

/**
  * @brief 丟棄軟體 RX ring buffer 中尚未處理的所有 CAN frame。
  * @retval 無。
  * @note FDCAN ISR 可能同時更新 head，因此以保留原 PRIMASK 的臨界區同步。
  */
static void MotorCAN_ClearRxQueue(void)
{
  uint32_t primask = __get_PRIMASK();

  /* head 可能由 ISR 更新；保存 PRIMASK 可避免意外開啟呼叫端原本關閉的中斷。 */
  __disable_irq();
  motor_rx_tail = motor_rx_head;
  motor_rx_overflow = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
  * @brief 停止並重新初始化指定 FDCAN bus，以切換 500K／1M bitrate。
  * @param bus 1-based bus 編號。
  * @param bitrate_kbps 目標速率，只接受 500 或 1000。
  * @retval 1 表示重新初始化、filter、start 與 notification 全部成功；否則為 0。
  * @note 成功後會清除舊 RX frame、bus error 與 recovery 狀態。
  */
static uint8_t MotorCAN_ReconfigureBus(uint8_t bus, uint16_t bitrate_kbps)
{
  FDCAN_HandleTypeDef *handle;
  uint32_t prescaler;

  if ((!motor_can_ready) || (!MotorCAN_IsValidBus(bus)) ||
      (!MotorCAN_IsSupportedBitrate(bitrate_kbps)))
  {
    return 0U;
  }

  if (motor_bus_online[bus - 1U] &&
      (motor_bus_bitrate_kbps[bus - 1U] == bitrate_kbps))
  {
    MotorCAN_ClearRxQueue();
    return 1U;
  }

  handle = motor_can_handles[bus - 1U];
  prescaler = (bitrate_kbps == MOTOR_CAN_RATE_500_KBPS) ?
              MOTOR_CAN_PRESCALER_500K : MOTOR_CAN_PRESCALER_1M;

  /*
   * 切換 bitrate 前先停用 notification，再 DeInit/Init 整個 FDCAN instance。
   * 只修改 prescaler，其他 timing segment 沿用 CubeMX 的 1 Mbit/s 設定比例。
   * 完成後清掉舊 RX frame，避免把切換前的回覆誤認成目前 state 的回覆。
   */
  (void)HAL_FDCAN_DeactivateNotification(handle, MOTOR_CAN_NOTIFICATIONS);
  motor_bus_online[bus - 1U] = 0U;
  if (HAL_FDCAN_DeInit(handle) != HAL_OK)
  {
    return 0U;
  }

  handle->Init.NominalPrescaler = prescaler;
  handle->Init.DataPrescaler = prescaler;
  if (HAL_FDCAN_Init(handle) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(handle,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FDCAN_Start(handle) != HAL_OK)
  {
    return 0U;
  }
  if (HAL_FDCAN_ActivateNotification(handle, MOTOR_CAN_NOTIFICATIONS, 0U) != HAL_OK)
  {
    return 0U;
  }

  motor_bus_bitrate_kbps[bus - 1U] = bitrate_kbps;
  motor_bus_online[bus - 1U] = 1U;
  motor_bus_error_flags[bus - 1U] = 0U;
  motor_bus_recovery_pending[bus - 1U] = 0U;
  motor_bus_recovery_deadline[bus - 1U] = 0U;
  MotorCAN_ClearRxQueue();
  return 1U;
}

/**
  * @brief 依目前 bitrate 重啟一條 error-passive 或 bus-off 的 FDCAN bus。
  * @param bus 1-based bus 編號。
  * @retval 1 表示 bus 已重新 start 並啟用 notification；0 表示復原失敗。
  * @note 一般 Stop/Start 無法復原時，會改走完整 ReconfigureBus 流程。
  */
static uint8_t MotorCAN_RestartBus(uint8_t bus)
{
  FDCAN_HandleTypeDef *handle;
  HAL_FDCAN_StateTypeDef state;
  uint16_t bitrate_kbps;

  if ((!MotorCAN_IsValidBus(bus)) ||
      (!MotorCAN_IsSupportedBitrate(motor_bus_bitrate_kbps[bus - 1U])))
  {
    return 0U;
  }

  handle = motor_can_handles[bus - 1U];
  bitrate_kbps = motor_bus_bitrate_kbps[bus - 1U];

  /*
   * Bus-off 必須讓 FDCAN 重新進入再離開 INIT，控制器才會開始復原。
   * 這裡在主迴圈執行，避免在中斷 callback 內呼叫會等待硬體狀態的 HAL 函式。
   */
  (void)HAL_FDCAN_DeactivateNotification(handle, MOTOR_CAN_NOTIFICATIONS);
  state = HAL_FDCAN_GetState(handle);

  if ((state == HAL_FDCAN_STATE_BUSY) &&
      (HAL_FDCAN_Stop(handle) != HAL_OK))
  {
    /* 一般 Stop 失敗時完整重建該路 FDCAN，並保留原本 bitrate。 */
    return MotorCAN_ReconfigureBus(bus, bitrate_kbps);
  }
  if ((state != HAL_FDCAN_STATE_BUSY) &&
      (state != HAL_FDCAN_STATE_READY))
  {
    return MotorCAN_ReconfigureBus(bus, bitrate_kbps);
  }

  if (HAL_FDCAN_Start(handle) != HAL_OK)
  {
    return MotorCAN_ReconfigureBus(bus, bitrate_kbps);
  }
  if (HAL_FDCAN_ActivateNotification(handle,
                                     MOTOR_CAN_NOTIFICATIONS, 0U) != HAL_OK)
  {
    (void)HAL_FDCAN_Stop(handle);
    return MotorCAN_ReconfigureBus(bus, bitrate_kbps);
  }

  motor_bus_online[bus - 1U] = 1U;
  return 1U;
}

/**
  * @brief 原子地取出並清除某一路由 ISR 累積的 FDCAN 錯誤旗標。
  * @param index 0-based bus 陣列索引。
  * @return 此次取出的 HAL FDCAN error-status bit mask。
  * @note 讀取與清零必須在同一臨界區，避免遺失同步到達的新錯誤。
  */
static uint32_t MotorCAN_TakeBusErrorFlags(uint8_t index)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t flags;

  /* ISR 可能同時加入新錯誤，必須以臨界區完成讀取及清除。 */
  __disable_irq();
  flags = motor_bus_error_flags[index];
  motor_bus_error_flags[index] = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return flags;
}

/**
  * @brief 清空目前高階操作的 context，回到 IDLE。
  * @retval 無。
  * @note 不會改動 FDCAN handle、bus bitrate、RX queue 或 recovery 狀態。
  */
static void MotorCAN_ResetOperation(void)
{
  /* 僅重設高階操作；bus handle、bitrate、queue 與 recovery 狀態都保留。 */
  memset(&motor_context, 0, sizeof(motor_context));
  motor_context.operation = MOTOR_CAN_OPERATION_NONE;
  motor_context.state = MOTOR_STATE_IDLE;
}

/**
  * @brief 將一筆完成或錯誤事件加入 Motor CAN event ring buffer。
  * @param event 要複製進 queue 的事件；呼叫端必須提供有效指標。
  * @retval 無。
  * @note queue 滿時丟棄最舊事件，保留最新完成結果或錯誤。
  */
static void MotorCAN_PushEvent(const MotorCAN_Event *event)
{
  uint8_t next = (uint8_t)((motor_event_head + 1U) % MOTOR_CAN_EVENT_QUEUE_SIZE);

  if (next == motor_event_tail)
  {
    /* 保留最新結果；queue 滿時丟棄最舊的一筆。 */
    motor_event_tail = (uint8_t)((motor_event_tail + 1U) % MOTOR_CAN_EVENT_QUEUE_SIZE);
  }

  motor_event_queue[motor_event_head] = *event;
  motor_event_head = next;
}

/**
  * @brief 依目前 operation context 建立一筆 MOTOR_CAN_EVENT_ERROR。
  * @param error 要回報給 USB command 層的失敗原因。
  * @retval 無。
  * @note 事件會帶入目前 bus、id、old_id 與 new_id，方便上位機定位操作。
  */
static void MotorCAN_PushError(MotorCAN_Error error)
{
  MotorCAN_Event event = {0};

  event.type = MOTOR_CAN_EVENT_ERROR;
  event.error = error;
  event.bus = motor_context.bus;
  event.id = motor_context.id;
  event.old_id = motor_context.old_id;
  event.new_id = motor_context.new_id;
  MotorCAN_PushEvent(&event);
}

/**
  * @brief 組出一個 Servo42ES Classic CAN frame，附加 checksum 後送入 FDCAN TX FIFO。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達的 Standard CAN ID。
  * @param body 不含 checksum 的 command 與參數 bytes。
  * @param body_length body 長度，必須為 1..7，保留最後一 byte 給 checksum。
  * @retval 1 表示 HAL 已接受 frame；0 表示輸入無效、bus 離線、FIFO 滿或 HAL 失敗。
  */
static uint8_t MotorCAN_SendBody(uint8_t bus, uint16_t id,
                                 const uint8_t *body, uint8_t body_length)
{
  FDCAN_TxHeaderTypeDef header = {0};
  uint8_t payload[8] = {0};
  uint16_t sum = id;
  uint8_t index;

  if ((!motor_can_ready) || (!MotorCAN_IsValidBus(bus)) ||
      (!motor_bus_online[bus - 1U]) ||
      (!MotorCAN_IsValidNodeId(id)) || (body == NULL) ||
      (body_length == 0U) || (body_length > 7U))
  {
    return 0U;
  }

  /* Servo42ES checksum = CAN ID + command/body bytes，取最低 8 bits。 */
  for (index = 0U; index < body_length; index++)
  {
    payload[index] = body[index];
    sum = (uint16_t)(sum + body[index]);
  }
  payload[body_length] = (uint8_t)(sum & 0xFFU);

  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  /* Classic CAN 的 0..8 bytes DLC 編碼與 byte 數相同。 */
  header.DataLength = (uint32_t)(body_length + 1U);
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  if (HAL_FDCAN_GetTxFifoFreeLevel(motor_can_handles[bus - 1U]) == 0U)
  {
    return 0U;
  }

  return (HAL_FDCAN_AddMessageToTxFifoQ(motor_can_handles[bus - 1U],
                                         &header, payload) == HAL_OK) ? 1U : 0U;
}

/**
  * @brief 送出 Servo42ES 0x40 版本查詢。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendReadVersion(uint8_t bus, uint16_t id)
{
  const uint8_t body[] = {0x40U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 對 old_id 送出 0x8B 命令，要求改成 new_id。
  * @param bus 1-based bus 編號。
  * @param old_id 目前用來定址馬達的 ID。
  * @param new_id 要寫入馬達的 11-bit 新 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendSetId(uint8_t bus, uint16_t old_id, uint16_t new_id)
{
  const uint8_t body[] = {
    0x8BU,
    (uint8_t)((new_id >> 8) & 0xFFU),
    (uint8_t)(new_id & 0xFFU)
  };
  return MotorCAN_SendBody(bus, old_id, body, sizeof(body));
}

/**
  * @brief 送出 0x8A 命令，把馬達 CAN bitrate 設成 1 Mbit/s。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendSetBitrate1M(uint8_t bus, uint16_t id)
{
  /* MKS 手冊：0x8A 的參數 0x03 代表 1 Mbit/s。 */
  const uint8_t body[] = {0x8AU, 0x03U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x60/0x01，要求馬達把目前參數寫入內部非揮發性儲存。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendSave(uint8_t bus, uint16_t id)
{
  const uint8_t body[] = {0x60U, 0x01U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x41 軟體重啟命令，使馬達重新載入已儲存設定。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendReset(uint8_t bus, uint16_t id)
{
  /* MKS 手冊：0x41 只重啟控制器，不會清除已儲存的參數。 */
  const uint8_t body[] = {0x41U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0xF3 Enable／Disable 命令。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @param enable 非 0 表示 Enable；0 表示 Disable。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendEnable(uint8_t bus, uint16_t id, uint8_t enable)
{
  const uint8_t body[] = {0xF3U, enable ? 0x01U : 0x00U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x82 命令，把 Servo42ES 工作模式設為 CAN bus mode。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendSetBusMode(uint8_t bus, uint16_t id)
{
  /* Servo42ES 的工作模式 0x05 代表 CAN bus mode。 */
  const uint8_t body[] = {0x82U, 0x05U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x9E homing switch-level 設定，把 origin switch 設為 active-low。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  * @note limit/right switch 欄位使用 0xFF，要求馬達保留既有設定。
  */
static uint8_t MotorCAN_SendSetHomeSwitchLevel(uint8_t bus, uint16_t id)
{
  /* 保留 limit/right switch 設定，只把 origin switch 設為 active-low。 */
  const uint8_t body[] = {0x9EU, 0xFFU, 0x00U, 0xFFU};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x95 homing 模式、方向與高低速設定。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  * @note 速度與方向讀自 motor_context，由 MotorCAN_StartHome() 先行驗證。
  */
static uint8_t MotorCAN_SendSetHomeParameters(uint8_t bus, uint16_t id)
{
  const uint16_t high_speed = motor_context.home_high_speed_rpm;
  const uint16_t low_speed = motor_context.home_low_speed_rpm;
  const uint8_t body[] = {
    0x95U,
    0x00U, /* 0x00：使用 origin switch homing。 */
    motor_context.home_direction,
    (uint8_t)((high_speed >> 8) & 0xFFU),
    (uint8_t)(high_speed & 0xFFU),
    (uint8_t)((low_speed >> 8) & 0xFFU),
    (uint8_t)(low_speed & 0xFFU)
  };
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x96 homing torque 與找到原點後的位置 offset。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  * @note signed offset 以 int32_t two's-complement bit pattern 依 big-endian 順序送出。
  */
static uint8_t MotorCAN_SendSetHomeOffset(uint8_t bus, uint16_t id)
{
  const uint32_t offset = (uint32_t)motor_context.home_origin_offset;
  const uint8_t body[] = {
    0x96U,
    (uint8_t)((MOTOR_CAN_HOME_TORQUE_MA >> 8) & 0xFFU),
    (uint8_t)(MOTOR_CAN_HOME_TORQUE_MA & 0xFFU),
    (uint8_t)((offset >> 24) & 0xFFU),
    (uint8_t)((offset >> 16) & 0xFFU),
    (uint8_t)((offset >> 8) & 0xFFU),
    (uint8_t)(offset & 0xFFU)
  };
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x97 homing trigger mode 與馬達內部 timeout。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  * @note trigger mode 0 表示只在後續收到 0x91 時執行 homing。
  */
static uint8_t MotorCAN_SendSetHomeTrigger(uint8_t bus, uint16_t id)
{
  const uint32_t timeout = motor_context.home_timeout_ms;
  const uint8_t body[] = {
    0x97U,
    0x00U, /* 0x00：只在收到 0x91 命令時執行 homing。 */
    (uint8_t)((timeout >> 24) & 0xFFU),
    (uint8_t)((timeout >> 16) & 0xFFU),
    (uint8_t)((timeout >> 8) & 0xFFU),
    (uint8_t)(timeout & 0xFFU)
  };
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出 0x91/0x00，正式啟動 origin-return homing。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendExecuteHome(uint8_t bus, uint16_t id)
{
  /* 0x00 執行 origin return；馬達會回報 start、complete 或 timeout。 */
  const uint8_t body[] = {0x91U, 0x00U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 送出固定 30 RPM、加速度 2、執行 500 ms 的 0xF6 測試命令。
  * @param bus 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @retval 1 表示 frame 已排入 TX FIFO；0 表示送出失敗。
  */
static uint8_t MotorCAN_SendTimedTest(uint8_t bus, uint16_t id)
{
  const uint32_t runtime = MOTOR_CAN_TEST_RUNTIME_10MS;
  const uint8_t body[] = {
    0xF6U,
    (uint8_t)((MOTOR_CAN_TEST_SPEED_RPM >> 8) & 0x0FU),
    (uint8_t)(MOTOR_CAN_TEST_SPEED_RPM & 0xFFU),
    MOTOR_CAN_TEST_ACCELERATION,
    (uint8_t)((runtime >> 16) & 0xFFU),
    (uint8_t)((runtime >> 8) & 0xFFU),
    (uint8_t)(runtime & 0xFFU)
  };
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

/**
  * @brief 判斷目前 operation 是否會造成實際馬達移動。
  * @retval 1 表示目前為 TEST 或 HOME；其他操作回傳 0。
  * @note EMS 與錯誤處理用此判斷是否需要主動停止並 Disable 馬達。
  */
static uint8_t MotorCAN_IsMotionOperation(void)
{
  return ((motor_context.operation == MOTOR_CAN_OPERATION_TEST) ||
          (motor_context.operation == MOTOR_CAN_OPERATION_HOME)) ? 1U : 0U;
}

/**
  * @brief 停止目前 TEST／HOME，最後一律送出 Disable。
  * @param emergency 非 0 時使用 0xF7 立即停止；0 時 TEST 會以速度 0 減速停止。
  * @retval 無。
  * @note HOME 無法安全推斷當下方向，即使 emergency=0 也使用 0xF7 立即停止。
  */
static void MotorCAN_StopActiveMotion(uint8_t emergency)
{
  if ((!MotorCAN_IsMotionOperation()) ||
      (!MotorCAN_IsValidNodeId(motor_context.id)))
  {
    return;
  }

  if (emergency || (motor_context.operation == MOTOR_CAN_OPERATION_HOME))
  {
    /* Homing 方向未知，失敗時一律用 0xF7 立即停止，再解除使能。 */
    const uint8_t stop_body[] = {0xF7U};
    (void)MotorCAN_SendBody(motor_context.bus, motor_context.id,
                            stop_body, sizeof(stop_body));
  }
  else
  {
    /* 速度設為 0 並以 acc=2 減速停止。 */
    const uint8_t stop_body[] = {0xF6U, 0x00U, 0x00U,
                                 MOTOR_CAN_TEST_ACCELERATION};
    (void)MotorCAN_SendBody(motor_context.bus, motor_context.id,
                            stop_body, sizeof(stop_body));
  }

  (void)MotorCAN_SendEnable(motor_context.bus, motor_context.id, 0U);
}

/**
  * @brief 驗證一筆 Servo42ES 回覆的長度與 checksum。
  * @param frame 待驗證的軟體 RX frame。
  * @retval 1 表示格式與 checksum 正確；0 表示指標、長度或 checksum 無效。
  */
static uint8_t MotorCAN_FrameChecksumIsValid(const MotorCAN_RxFrame *frame)
{
  uint16_t sum;
  uint8_t index;

  if ((frame == NULL) || (frame->length < 2U) || (frame->length > 8U))
  {
    return 0U;
  }

  /* 接收格式與送出格式相同：最後一個 byte 是 ID 與前面 payload 的低 8-bit 和。 */
  sum = frame->id;
  for (index = 0U; index < (uint8_t)(frame->length - 1U); index++)
  {
    sum = (uint16_t)(sum + frame->data[index]);
  }

  return (((uint8_t)(sum & 0xFFU)) == frame->data[frame->length - 1U]) ? 1U : 0U;
}

/**
  * @brief 將 0x40 version reply 轉成 INFO 完成事件並結束目前操作。
  * @param frame 已通過 checksum、bus、ID 與 command 比對的版本回覆。
  * @retval 無。
  * @note 回覆長度足夠時會拆出 hardware nibble 與三段 firmware version。
  */
static void MotorCAN_CompleteInfo(const MotorCAN_RxFrame *frame)
{
  MotorCAN_Event event = {0};

  event.type = MOTOR_CAN_EVENT_INFO;
  event.bus = frame->bus;
  event.id = frame->id;
  if (frame->length >= 6U)
  {
    event.hardware_version = frame->data[1] & 0x0FU;
    event.firmware_version[0] = frame->data[2];
    event.firmware_version[1] = frame->data[3];
    event.firmware_version[2] = frame->data[4];
  }
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

/**
  * @brief 產生 ID_CHANGED 完成事件並清空 SET_ID state machine。
  * @retval 無。
  * @note 呼叫前必須已用新 ID 收到 0x40 reply，代表儲存後驗證成功。
  */
static void MotorCAN_CompleteIdChange(void)
{
  MotorCAN_Event event = {0};

  event.type = MOTOR_CAN_EVENT_ID_CHANGED;
  event.bus = motor_context.bus;
  event.id = motor_context.new_id;
  event.old_id = motor_context.old_id;
  event.new_id = motor_context.new_id;
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

/**
  * @brief 產生 MOTOR_RATE_1M 完成事件並清空 provisioning state machine。
  * @retval 無。
  * @note 呼叫時 STM32 與馬達都應已在 1 Mbit/s，且版本探測成功。
  */
static void MotorCAN_CompleteProvision1M(void)
{
  MotorCAN_Event event = {0};

  event.type = MOTOR_CAN_EVENT_MOTOR_RATE_1M;
  event.bus = motor_context.bus;
  event.id = motor_context.id;
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

/**
  * @brief 完成短時間 TEST，Disable 馬達並產生 TEST_FINISHED 事件。
  * @retval 無。
  */
static void MotorCAN_CompleteTest(void)
{
  MotorCAN_Event event = {0};

  (void)MotorCAN_SendEnable(motor_context.bus, motor_context.id, 0U);
  event.type = MOTOR_CAN_EVENT_TEST_FINISHED;
  event.bus = motor_context.bus;
  event.id = motor_context.id;
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

/**
  * @brief 完成 HOME 並產生 HOME_FINISHED 事件。
  * @retval 無。
  * @note 不會 Disable 馬達，讓馬達維持剛建立的原點位置。
  */
static void MotorCAN_CompleteHome(void)
{
  MotorCAN_Event event = {0};

  /* Homing 完成後保留使能，讓馬達維持剛建立的原點位置。 */
  event.type = MOTOR_CAN_EVENT_HOME_FINISHED;
  event.bus = motor_context.bus;
  event.id = motor_context.id;
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

/**
  * @brief 統一處理目前非同步操作失敗、必要停止、錯誤事件與 context 清理。
  * @param error 原始失敗原因。
  * @param stop_motion 非 0 時先停止並 Disable TEST／HOME 馬達。
  * @retval 無。
  * @note PROVISION_1M 失敗時會優先恢復 STM32 bus 為 1M；恢復失敗會改報 RECONFIG_FAILED。
  */
static void MotorCAN_FailOperation(MotorCAN_Error error, uint8_t stop_motion)
{
  /*
   * 動作類操作失敗時先停止並 Disable 馬達，再回報錯誤。
   * PROVISION_1M 額外保證 STM32 端盡量回到系統預設 1 Mbit/s，否則下一個
   * 一般命令會在錯誤 bitrate 上送出。
   */
  if (stop_motion)
  {
    MotorCAN_StopActiveMotion(0U);
  }

  if ((motor_context.operation == MOTOR_CAN_OPERATION_PROVISION_1M) &&
      (motor_bus_bitrate_kbps[motor_context.bus - 1U] != MOTOR_CAN_RATE_1M_KBPS) &&
      (!MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS)))
  {
    /* 恢復 1 Mbit/s 比原始錯誤更重要，回報 bus 尚未回到正常系統速率。 */
    error = MOTOR_CAN_ERROR_RECONFIG_FAILED;
  }
  MotorCAN_PushError(error);
  MotorCAN_ResetOperation();
}

/**
  * @brief 處理一次 state command 的送出結果，成功時設定下一 state 與 deadline。
  * @param sent 前一個 MotorCAN_Send*() 的布林結果。
  * @param next_state frame 成功送出後要等待的下一個 state。
  * @param timeout_ms 下一 state 可等待的時間。
  * @retval 無。
  * @note 送出失敗會立即結束目前操作並回報 MOTOR_CAN_ERROR_TX。
  */
static void MotorCAN_SendOrFail(uint8_t sent, MotorCAN_State next_state,
                                uint32_t timeout_ms)
{
  /* 集中處理「送出成功後切 state + 設 deadline」這個每一步都會用到的模式。 */
  if (!sent)
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_TX,
                           MotorCAN_IsMotionOperation());
    return;
  }

  motor_context.state = next_state;
  motor_context.deadline = HAL_GetTick() + timeout_ms;
}

/**
  * @brief 將 STM32 bus 切到 1M，立即以 0x40 探測 provisioning 目標。
  * @retval 無。
  * @note 用於 500K 上的 bitrate ACK 可能因馬達立即切速而遺失的情況。
  */
static void MotorCAN_SwitchProvisionTo1MAndProbe(void)
{
  /*
   * 馬達收到 0x8A 後可能立即改用 1 Mbit/s，導致 500K 上的 ACK 消失。
   * 因此 ACK timeout 不直接判定失敗，而是切換 STM32 bitrate 並實際探測馬達。
   */
  if (!MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS))
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_RECONFIG_FAILED, 0U);
    return;
  }

  MotorCAN_SendOrFail(
    MotorCAN_SendReadVersion(motor_context.bus, motor_context.id),
    MOTOR_STATE_RATE_FIND_1M, MOTOR_CAN_PROBE_TIMEOUT_MS);
}

/**
  * @brief 將 STM32 bus 切到 1M，並等待馬達 reset 後完成開機。
  * @retval 無。
  * @note 只設定 WAIT_REBOOT state/deadline；真正的版本探測由 timeout handler 送出。
  */
static void MotorCAN_SwitchProvisionTo1MAndWaitForRestart(void)
{
  /* 馬達 reset 後需要開機時間；先切回 1M，再由 timeout handler 延遲探測。 */
  if (!MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS))
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_RECONFIG_FAILED, 0U);
    return;
  }

  motor_context.state = MOTOR_STATE_RATE_WAIT_REBOOT_1M;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_RESTART_DELAY_MS;
}

/**
  * @brief 依目前 state 消化一筆有效 RX frame，推進非同步操作。
  * @param frame 從 ISR RX queue 取出的 frame。
  * @retval 無。
  * @note 會先驗證 checksum、目前 bus 與 active operation，再比對 ID/command/status。
  */
static void MotorCAN_HandleFrame(const MotorCAN_RxFrame *frame)
{
  /*
   * 只讓 checksum 正確、來自目前 bus 的 frame 進入 state machine。
   * 同一條 bus 上其他馬達的合法流量仍可能進入 RX queue，各 state 再用
   * frame->id 與 command byte 精確比對，無關 frame 直接忽略。
   *
   * 主要流程：
   *   INFO          0x40 reply -> complete
   *   SET_ID        probe old -> probe new -> 0x8B -> locate active ID
   *                 -> 0x60 save -> delay -> 0x40 verify
   *   TEST          probe -> 0xF3 enable -> 0xF6 timed run -> disable
   *   HOME          probe -> 0x82 -> 0x9E -> 0x95 -> 0x96 -> 0x97
   *                 -> 0x60 save -> 0xF3 enable -> 0x91 execute
   *   PROVISION_1M  probe@500K -> 0x8A -> 0x60 -> 0x41 reset
   *                 -> switch STM32 to 1M -> 0x40 verify
   */
  if ((!MotorCAN_FrameChecksumIsValid(frame)) ||
      (frame->bus != motor_context.bus) ||
      (motor_context.operation == MOTOR_CAN_OPERATION_NONE))
  {
    return;
  }

  switch (motor_context.state)
  {
    /* INFO：version reply 本身就是最終結果。 */
    case MOTOR_STATE_INFO_WAIT:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteInfo(frame);
      }
      break;

    /* SET_ID 第一階段：舊 ID 必須存在。 */
    case MOTOR_STATE_SET_PROBE_OLD:
      if ((frame->id == motor_context.old_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendReadVersion(motor_context.bus, motor_context.new_id),
          MOTOR_STATE_SET_PROBE_NEW, MOTOR_CAN_PROBE_TIMEOUT_MS);
      }
      break;

    /* 新 ID 若有回覆代表已被占用；沒有回覆則由 timeout handler 送 0x8B。 */
    case MOTOR_STATE_SET_PROBE_NEW:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_FailOperation(MOTOR_CAN_ERROR_NEW_ID_IN_USE, 0U);
      }
      break;

    /* 馬達可能用舊 ID 或新 ID 回 ACK，兩者都接受。 */
    case MOTOR_STATE_SET_WAIT_ACK:
      if (((frame->id == motor_context.old_id) ||
           (frame->id == motor_context.new_id)) &&
          (frame->data[0] == 0x8BU) && (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 0U);
          break;
        }
        motor_context.set_ack_received = 1U;
        MotorCAN_SendOrFail(
          MotorCAN_SendReadVersion(motor_context.bus, motor_context.new_id),
          MOTOR_STATE_SET_FIND_NEW, MOTOR_CAN_PROBE_TIMEOUT_MS);
      }
      break;

    /* ACK 可能遺失，實際探測新 ID 才是改 ID 是否生效的主要判據。 */
    case MOTOR_STATE_SET_FIND_NEW:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        motor_context.active_id = motor_context.new_id;
        MotorCAN_SendOrFail(
          MotorCAN_SendSave(motor_context.bus, motor_context.active_id),
          MOTOR_STATE_SET_WAIT_SAVE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    /* 新 ID 找不到時回查舊 ID，用來區分 ACK 遺失與裝置拒絕。 */
    case MOTOR_STATE_SET_FIND_OLD:
      if ((frame->id == motor_context.old_id) && (frame->data[0] == 0x40U))
      {
        if (!motor_context.set_ack_received)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 0U);
          break;
        }
        motor_context.active_id = motor_context.old_id;
        MotorCAN_SendOrFail(
          MotorCAN_SendSave(motor_context.bus, motor_context.active_id),
          MOTOR_STATE_SET_WAIT_SAVE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    /* 對目前實際在線的 ID 儲存設定，接著留 200 ms 給內部 flash 完成。 */
    case MOTOR_STATE_SET_WAIT_SAVE:
      if (((frame->id == motor_context.active_id) ||
           (frame->id == motor_context.old_id) ||
           (frame->id == motor_context.new_id)) &&
          (frame->data[0] == 0x60U) && (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_SAVE_FAILED, 0U);
          break;
        }
        motor_context.state = MOTOR_STATE_SET_WAIT_VERIFY_DELAY;
        motor_context.deadline = HAL_GetTick() + MOTOR_CAN_VERIFY_DELAY_MS;
      }
      break;

    /* 最後只接受新 ID 的 version reply，確認設定已套用且可通訊。 */
    case MOTOR_STATE_SET_WAIT_VERIFY:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteIdChange();
      }
      break;

    /* PROVISION_1M 從 500K 開始，先確認出廠 bitrate 下能找到目標馬達。 */
    case MOTOR_STATE_RATE_PROBE_500:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendSetBitrate1M(motor_context.bus, motor_context.id),
          MOTOR_STATE_RATE_WAIT_SET_ACK_500, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    /* 若馬達仍在 500K 回 ACK，立即送 save；若已切 1M，timeout 會改速率探測。 */
    case MOTOR_STATE_RATE_WAIT_SET_ACK_500:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x8AU) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 0U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSave(motor_context.bus, motor_context.id),
          MOTOR_STATE_RATE_WAIT_SAVE_500, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_RATE_WAIT_SAVE_500:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x60U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_SAVE_FAILED, 0U);
          break;
        }
        motor_context.rate_save_ack_received = 1U;
        MotorCAN_SendOrFail(
          MotorCAN_SendReset(motor_context.bus, motor_context.id),
          MOTOR_STATE_RATE_WAIT_RESET_500, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_RATE_WAIT_RESET_500:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x41U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 0U);
          break;
        }
        MotorCAN_SwitchProvisionTo1MAndWaitForRestart();
      }
      break;

    /* 1M 探測成功後，依先前是否收到 save ACK 決定完成或補送 save。 */
    case MOTOR_STATE_RATE_FIND_1M:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        if (motor_context.rate_save_ack_received)
        {
          MotorCAN_CompleteProvision1M();
        }
        else
        {
          /* 有些韌體會立刻套用新 bitrate，因此改在 1 Mbit/s 補送儲存。 */
          MotorCAN_SendOrFail(
            MotorCAN_SendSave(motor_context.bus, motor_context.id),
            MOTOR_STATE_RATE_WAIT_SAVE_1M, MOTOR_CAN_COMMAND_TIMEOUT_MS);
        }
      }
      break;

    case MOTOR_STATE_RATE_WAIT_SAVE_1M:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x60U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_SAVE_FAILED, 0U);
          break;
        }
        motor_context.rate_save_ack_received = 1U;
        MotorCAN_SendOrFail(
          MotorCAN_SendReset(motor_context.bus, motor_context.id),
          MOTOR_STATE_RATE_WAIT_RESET_1M, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_RATE_WAIT_RESET_1M:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x41U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 0U);
          break;
        }
        motor_context.state = MOTOR_STATE_RATE_WAIT_REBOOT_1M;
        motor_context.deadline = HAL_GetTick() + MOTOR_CAN_RESTART_DELAY_MS;
      }
      break;

    /* reset 後的 1M version reply 是 provisioning 成功的最終依據。 */
    case MOTOR_STATE_RATE_WAIT_VERIFY:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteProvision1M();
      }
      break;

    /* TEST 先 probe，避免對不存在的 ID 直接送運動命令。 */
    case MOTOR_STATE_TEST_PROBE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendEnable(motor_context.bus, motor_context.id, 1U),
          MOTOR_STATE_TEST_WAIT_ENABLE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_TEST_WAIT_ENABLE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0xF3U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendTimedTest(motor_context.bus, motor_context.id),
          MOTOR_STATE_TEST_WAIT_RUN, MOTOR_CAN_TEST_TIMEOUT_MS);
      }
      break;

    /* 0xF6 status 1 表示已開始、2 表示完成；start reply 會刷新 deadline。 */
    case MOTOR_STATE_TEST_WAIT_RUN:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0xF6U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] == 2U)
        {
          MotorCAN_CompleteTest();
        }
        else if (frame->data[1] == 1U)
        {
          motor_context.deadline = HAL_GetTick() + MOTOR_CAN_TEST_TIMEOUT_MS;
        }
        else
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
        }
      }
      break;

    /* HOME 同樣先 probe；後續每一項設定都必須收到 status=1 才繼續。 */
    case MOTOR_STATE_HOME_PROBE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendSetBusMode(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_MODE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_MODE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x82U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSetHomeSwitchLevel(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_SWITCH_LEVEL, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_SWITCH_LEVEL:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x9EU) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSetHomeParameters(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_PARAMETERS, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_PARAMETERS:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x95U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSetHomeOffset(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_OFFSET, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_OFFSET:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x96U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSetHomeTrigger(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_TRIGGER, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_TRIGGER:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x97U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendSave(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_SAVE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_SAVE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x60U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_SAVE_FAILED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendEnable(motor_context.bus, motor_context.id, 1U),
          MOTOR_STATE_HOME_WAIT_ENABLE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_HOME_WAIT_ENABLE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0xF3U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] != 1U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
          break;
        }
        MotorCAN_SendOrFail(
          MotorCAN_SendExecuteHome(motor_context.bus, motor_context.id),
          MOTOR_STATE_HOME_WAIT_EXECUTE,
          motor_context.home_timeout_ms + MOTOR_CAN_HOME_TIMEOUT_GUARD_MS);
      }
      break;

    /* 0x91 status：1=start、2=complete、3=馬達內部 timeout。 */
    case MOTOR_STATE_HOME_WAIT_EXECUTE:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x91U) &&
          (frame->length >= 3U))
      {
        if (frame->data[1] == 1U)
        {
          /* start 回覆後重新計時，避免前段傳輸時間吃掉 homing timeout。 */
          motor_context.deadline = HAL_GetTick() +
            motor_context.home_timeout_ms + MOTOR_CAN_HOME_TIMEOUT_GUARD_MS;
        }
        else if (frame->data[1] == 2U)
        {
          MotorCAN_CompleteHome();
        }
        else if (frame->data[1] == 3U)
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_HOME_TIMEOUT, 1U);
        }
        else
        {
          MotorCAN_FailOperation(MOTOR_CAN_ERROR_DEVICE_REJECTED, 1U);
        }
      }
      break;

    default:
      break;
  }
}

/**
  * @brief 處理目前 state 的 deadline，執行 timeout、延遲或協定容錯分支。
  * @param now 目前 HAL tick，單位為 ms。
  * @retval 無。
  * @note 部分 timeout 是預期流程，例如確認新 ID 無人使用、等待 flash 或馬達重啟。
  */
static void MotorCAN_HandleTimeout(uint32_t now)
{
  /*
   * timeout 有兩種意義：
   * 1. 真正失敗，例如 probe 無回覆或 homing 超時。
   * 2. 協定容錯／延遲步驟，例如新 ID probe 無回覆代表可使用，或等待馬達重啟。
   * 所以各 state 必須分別決定下一步，不能一律回報 RESPONSE_TIMEOUT。
   */
  if ((motor_context.operation == MOTOR_CAN_OPERATION_NONE) ||
      (!MotorCAN_DeadlineReached(now, motor_context.deadline)))
  {
    return;
  }

  switch (motor_context.state)
  {
    case MOTOR_STATE_INFO_WAIT:
    case MOTOR_STATE_SET_PROBE_OLD:
    case MOTOR_STATE_TEST_PROBE:
    case MOTOR_STATE_HOME_PROBE:
    case MOTOR_STATE_RATE_PROBE_500:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_TARGET_NOT_FOUND,
                             MotorCAN_IsMotionOperation());
      break;

    case MOTOR_STATE_SET_PROBE_NEW:
      MotorCAN_SendOrFail(
        MotorCAN_SendSetId(motor_context.bus, motor_context.old_id,
                           motor_context.new_id),
        MOTOR_STATE_SET_WAIT_ACK, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      break;

    case MOTOR_STATE_SET_WAIT_ACK:
      /* ACK 可能遺失，但馬達已套用新 ID；改以實際回覆判斷。 */
      MotorCAN_SendOrFail(
        MotorCAN_SendReadVersion(motor_context.bus, motor_context.new_id),
        MOTOR_STATE_SET_FIND_NEW, MOTOR_CAN_PROBE_TIMEOUT_MS);
      break;

    case MOTOR_STATE_SET_FIND_NEW:
      MotorCAN_SendOrFail(
        MotorCAN_SendReadVersion(motor_context.bus, motor_context.old_id),
        MOTOR_STATE_SET_FIND_OLD, MOTOR_CAN_PROBE_TIMEOUT_MS);
      break;

    case MOTOR_STATE_SET_FIND_OLD:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_RESPONSE_TIMEOUT, 0U);
      break;

    case MOTOR_STATE_SET_WAIT_SAVE:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_SAVE_FAILED, 0U);
      break;

    case MOTOR_STATE_SET_WAIT_VERIFY_DELAY:
      MotorCAN_SendOrFail(
        MotorCAN_SendReadVersion(motor_context.bus, motor_context.new_id),
        MOTOR_STATE_SET_WAIT_VERIFY, MOTOR_CAN_PROBE_TIMEOUT_MS);
      break;

    case MOTOR_STATE_SET_WAIT_VERIFY:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_VERIFY_FAILED, 0U);
      break;

    case MOTOR_STATE_RATE_WAIT_SET_ACK_500:
      /* 馬達可能已立即切到 1 Mbit/s，改用新速率探測並補做儲存。 */
      MotorCAN_SwitchProvisionTo1MAndProbe();
      break;

    case MOTOR_STATE_RATE_WAIT_SAVE_500:
      /* 儲存 ACK 可能遺失；先重啟，再用 1 Mbit/s 判斷是否真的保存成功。 */
      MotorCAN_SendOrFail(
        MotorCAN_SendReset(motor_context.bus, motor_context.id),
        MOTOR_STATE_RATE_WAIT_RESET_500, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      break;

    case MOTOR_STATE_RATE_WAIT_RESET_500:
      MotorCAN_SwitchProvisionTo1MAndWaitForRestart();
      break;

    case MOTOR_STATE_RATE_FIND_1M:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_VERIFY_FAILED, 0U);
      break;

    case MOTOR_STATE_RATE_WAIT_SAVE_1M:
      MotorCAN_SendOrFail(
        MotorCAN_SendReset(motor_context.bus, motor_context.id),
        MOTOR_STATE_RATE_WAIT_RESET_1M, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      break;

    case MOTOR_STATE_RATE_WAIT_RESET_1M:
      motor_context.state = MOTOR_STATE_RATE_WAIT_REBOOT_1M;
      motor_context.deadline = HAL_GetTick() + MOTOR_CAN_RESTART_DELAY_MS;
      break;

    case MOTOR_STATE_RATE_WAIT_REBOOT_1M:
      MotorCAN_SendOrFail(
        MotorCAN_SendReadVersion(motor_context.bus, motor_context.id),
        MOTOR_STATE_RATE_WAIT_VERIFY, MOTOR_CAN_PROBE_TIMEOUT_MS);
      break;

    case MOTOR_STATE_RATE_WAIT_VERIFY:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_VERIFY_FAILED, 0U);
      break;

    case MOTOR_STATE_TEST_WAIT_ENABLE:
    case MOTOR_STATE_TEST_WAIT_RUN:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_RESPONSE_TIMEOUT, 1U);
      break;

    case MOTOR_STATE_HOME_WAIT_MODE:
    case MOTOR_STATE_HOME_WAIT_SWITCH_LEVEL:
    case MOTOR_STATE_HOME_WAIT_PARAMETERS:
    case MOTOR_STATE_HOME_WAIT_OFFSET:
    case MOTOR_STATE_HOME_WAIT_TRIGGER:
    case MOTOR_STATE_HOME_WAIT_SAVE:
    case MOTOR_STATE_HOME_WAIT_ENABLE:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_RESPONSE_TIMEOUT, 1U);
      break;

    case MOTOR_STATE_HOME_WAIT_EXECUTE:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_HOME_TIMEOUT, 1U);
      break;

    default:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_RESPONSE_TIMEOUT,
                             MotorCAN_IsMotionOperation());
      break;
  }
}

/**
  * @brief 綁定兩條邏輯 CAN bus，設定接收 filter，啟動 FDCAN 與 RX/error notification。
  * @param can1_handle 邏輯 bus 1 的 HAL handle；本板對應 FDCAN3。
  * @param can2_handle 邏輯 bus 2 的 HAL handle；本板對應 FDCAN2。
  * @return MOTOR_CAN_STATUS_OK 表示兩路均已上線；輸入無效或任一路 HAL 初始化失敗則回錯誤。
  * @note 初始軟體 bitrate 設為 1 Mbit/s，且只接受 Standard data frame 到 FIFO0。
  */
MotorCAN_Status MotorCAN_Init(FDCAN_HandleTypeDef *can1_handle,
                              FDCAN_HandleTypeDef *can2_handle)
{
  uint8_t index;

  if ((can1_handle == NULL) || (can2_handle == NULL))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }

  /* 對外 bus 編號固定為 1/2；陣列索引則是 0/1。 */
  motor_can_handles[0] = can1_handle;
  motor_can_handles[1] = can2_handle;
  motor_bus_bitrate_kbps[0] = MOTOR_CAN_RATE_1M_KBPS;
  motor_bus_bitrate_kbps[1] = MOTOR_CAN_RATE_1M_KBPS;
  motor_bus_online[0] = 0U;
  motor_bus_online[1] = 0U;
  motor_bus_recovery_pending[0] = 0U;
  motor_bus_recovery_pending[1] = 0U;
  motor_bus_recovery_deadline[0] = 0U;
  motor_bus_recovery_deadline[1] = 0U;
  MotorCAN_ResetOperation();

  for (index = 0U; index < MOTOR_CAN_BUS_COUNT; index++)
  {
    /*
     * 接收所有 Standard data frame 到 FIFO0，讓同一套 state machine 可操作
     * 1..0x7FF 的任意節點；Extended 與 Remote frame 不屬於 Servo42ES protocol。
     */
    if (HAL_FDCAN_ConfigGlobalFilter(motor_can_handles[index],
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
      return MOTOR_CAN_STATUS_NOT_READY;
    }
    if (HAL_FDCAN_Start(motor_can_handles[index]) != HAL_OK)
    {
      return MOTOR_CAN_STATUS_NOT_READY;
    }
    if (HAL_FDCAN_ActivateNotification(motor_can_handles[index],
                                        MOTOR_CAN_NOTIFICATIONS, 0U) != HAL_OK)
    {
      return MOTOR_CAN_STATUS_NOT_READY;
    }
    motor_bus_online[index] = 1U;
  }

  motor_can_ready = 1U;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 在沒有 active operation 時切換指定 STM32 CAN bus 的 bitrate。
  * @param bus 1-based bus 編號。
  * @param bitrate_kbps 500 或 1000 kbit/s。
  * @return OK、NOT_READY、BUSY、INVALID_ARGUMENT 或 RECONFIG_FAILED。
  * @note 切換失敗時會盡力恢復原 bitrate。
  */
MotorCAN_Status MotorCAN_SetBusBitrate(uint8_t bus, uint16_t bitrate_kbps)
{
  uint16_t previous_bitrate;

  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) ||
      (!MotorCAN_IsSupportedBitrate(bitrate_kbps)))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  previous_bitrate = motor_bus_bitrate_kbps[bus - 1U];
  if (!MotorCAN_ReconfigureBus(bus, bitrate_kbps))
  {
    /* 重設失敗時嘗試回到切換前的速率，避免 bus 留在未初始化狀態。 */
    if (MotorCAN_IsSupportedBitrate(previous_bitrate))
    {
      (void)MotorCAN_ReconfigureBus(bus, previous_bitrate);
    }
    return MOTOR_CAN_STATUS_RECONFIG_FAILED;
  }
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 取得指定 STM32 CAN bus 目前的軟體追蹤 bitrate。
  * @param bus 1-based bus 編號。
  * @return 500、1000；bus 無效或目前離線時回傳 0。
  */
uint16_t MotorCAN_GetBusBitrate(uint8_t bus)
{
  if ((!MotorCAN_IsValidBus(bus)) || (!motor_bus_online[bus - 1U]))
  {
    return 0U;
  }
  return motor_bus_bitrate_kbps[bus - 1U];
}

/**
  * @brief 啟動「出廠 500K 馬達改成 1M」的非同步 provisioning 流程。
  * @param bus 目標 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @return 立即啟動狀態；OK 只表示已在 500K 送出第一筆 version probe。
  * @note 完整流程包含切速、寫入、儲存、reset、切回 1M 與版本驗證。
  */
MotorCAN_Status MotorCAN_StartProvision1M(uint8_t bus, uint16_t id)
{
  /*
   * Provisioning 一開始主動把 STM32 端切到 500K。任一步驟失敗時，
   * MotorCAN_FailOperation() 或這裡的 early-return 會盡力恢復為 1M。
   */
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(id)))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  MotorCAN_ResetOperation();
  motor_context.operation = MOTOR_CAN_OPERATION_PROVISION_1M;
  motor_context.bus = bus;
  motor_context.id = id;

  if (!MotorCAN_ReconfigureBus(bus, MOTOR_CAN_RATE_500_KBPS))
  {
    (void)MotorCAN_ReconfigureBus(bus, MOTOR_CAN_RATE_1M_KBPS);
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_RECONFIG_FAILED;
  }
  if (!MotorCAN_SendReadVersion(bus, id))
  {
    if (!MotorCAN_ReconfigureBus(bus, MOTOR_CAN_RATE_1M_KBPS))
    {
      MotorCAN_ResetOperation();
      return MOTOR_CAN_STATUS_RECONFIG_FAILED;
    }
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_TX_FAILED;
  }

  motor_context.state = MOTOR_STATE_RATE_PROBE_500;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_PROBE_TIMEOUT_MS;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 啟動非同步版本查詢。
  * @param bus 目標 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @return 立即啟動狀態；最終版本或錯誤由 MotorCAN_GetEvent() 回報。
  */
MotorCAN_Status MotorCAN_StartInfo(uint8_t bus, uint16_t id)
{
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(id)))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  MotorCAN_ResetOperation();
  motor_context.operation = MOTOR_CAN_OPERATION_INFO;
  motor_context.bus = bus;
  motor_context.id = id;
  if (!MotorCAN_SendReadVersion(bus, id))
  {
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_TX_FAILED;
  }
  motor_context.state = MOTOR_STATE_INFO_WAIT;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_PROBE_TIMEOUT_MS;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 啟動安全的非同步 CAN ID 修改流程。
  * @param bus 目標 1-based bus 編號。
  * @param old_id 馬達目前 ID。
  * @param new_id 尚未被其他馬達占用的新 ID。
  * @return 立即啟動狀態；OK 表示已送出 old_id probe。
  * @note 流程會確認舊 ID 存在、新 ID 未占用，修改後儲存並以新 ID 驗證。
  */
MotorCAN_Status MotorCAN_StartSetId(uint8_t bus, uint16_t old_id,
                                    uint16_t new_id)
{
  /*
   * 先 probe 舊 ID，再 probe 新 ID。確認目標存在且新 ID 未占用後，
   * timeout handler 才會真正送出修改命令，避免覆蓋同 bus 上的其他裝置。
   */
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(old_id)) ||
      (!MotorCAN_IsValidNodeId(new_id)) || (old_id == new_id))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  MotorCAN_ResetOperation();
  motor_context.operation = MOTOR_CAN_OPERATION_SET_ID;
  motor_context.bus = bus;
  motor_context.id = old_id;
  motor_context.old_id = old_id;
  motor_context.new_id = new_id;
  if (!MotorCAN_SendReadVersion(bus, old_id))
  {
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_TX_FAILED;
  }
  motor_context.state = MOTOR_STATE_SET_PROBE_OLD;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_PROBE_TIMEOUT_MS;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 啟動固定 30 RPM、500 ms 的非同步低速測試。
  * @param bus 目標 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @return 立即啟動狀態；EMS active、已有操作或傳送失敗時不會啟動。
  * @note 完成、EMS 中止或錯誤後都會 Disable 馬達。
  */
MotorCAN_Status MotorCAN_StartTest(uint8_t bus, uint16_t id)
{
  /* 運動操作在 EMS active 時直接拒絕，避免 state machine 被建立後才停止。 */
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(id)))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (EMS_IsStopActive())
  {
    return MOTOR_CAN_STATUS_EMS_ACTIVE;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  MotorCAN_ResetOperation();
  motor_context.operation = MOTOR_CAN_OPERATION_TEST;
  motor_context.bus = bus;
  motor_context.id = id;
  if (!MotorCAN_SendReadVersion(bus, id))
  {
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_TX_FAILED;
  }
  motor_context.state = MOTOR_STATE_TEST_PROBE;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_PROBE_TIMEOUT_MS;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 啟動使用 origin switch 的非同步 homing 設定與執行流程。
  * @param bus 目標 1-based bus 編號。
  * @param id 目標馬達 ID。
  * @param direction 0 為正向，1 為反向。
  * @param high_speed_rpm 高速尋找速度，允許 1..3000 RPM。
  * @param low_speed_rpm 低速精定位速度，允許 1..100 RPM 且不可高於 high speed。
  * @param origin_offset 找到 switch 後的 signed position offset。
  * @param timeout_ms 馬達內部 homing timeout，允許 1000..120000 ms。
  * @return 立即啟動狀態；最終完成、EMS 中止或錯誤由 event queue 回報。
  */
MotorCAN_Status MotorCAN_StartHome(uint8_t bus, uint16_t id,
                                   uint8_t direction,
                                   uint16_t high_speed_rpm,
                                   uint16_t low_speed_rpm,
                                   int32_t origin_offset,
                                   uint32_t timeout_ms)
{
  /* HOME 會寫入並儲存馬達參數，先完整驗證使用者輸入與 EMS 狀態。 */
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(id)) ||
      (direction > 1U) ||
      (high_speed_rpm < 1U) || (high_speed_rpm > 3000U) ||
      (low_speed_rpm < 1U) || (low_speed_rpm > 100U) ||
      (low_speed_rpm > high_speed_rpm) ||
      (timeout_ms < MOTOR_CAN_HOME_MIN_TIMEOUT_MS) ||
      (timeout_ms > MOTOR_CAN_HOME_MAX_TIMEOUT_MS))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (EMS_IsStopActive())
  {
    return MOTOR_CAN_STATUS_EMS_ACTIVE;
  }
  if (motor_context.operation != MOTOR_CAN_OPERATION_NONE)
  {
    return MOTOR_CAN_STATUS_BUSY;
  }

  MotorCAN_ResetOperation();
  motor_context.operation = MOTOR_CAN_OPERATION_HOME;
  motor_context.bus = bus;
  motor_context.id = id;
  motor_context.home_direction = direction;
  motor_context.home_high_speed_rpm = high_speed_rpm;
  motor_context.home_low_speed_rpm = low_speed_rpm;
  motor_context.home_origin_offset = origin_offset;
  motor_context.home_timeout_ms = timeout_ms;
  if (!MotorCAN_SendReadVersion(bus, id))
  {
    MotorCAN_ResetOperation();
    return MOTOR_CAN_STATUS_TX_FAILED;
  }
  motor_context.state = MOTOR_STATE_HOME_PROBE;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_PROBE_TIMEOUT_MS;
  return MOTOR_CAN_STATUS_OK;
}

/**
  * @brief 在主迴圈推進 Motor CAN 的 error recovery、EMS、RX frame 與 timeout。
  * @retval 無。
  * @note 必須持續非阻塞呼叫；執行順序刻意讓 bus error 與 EMS 優先於一般 frame。
  */
void MotorCAN_Process(void)
{
  MotorCAN_RxFrame frame;
  uint32_t now;
  uint8_t index;

  if (!motor_can_ready)
  {
    return;
  }

  /* 先把 ISR 偵測到的 queue overflow 轉成可由 USB 回報的事件。 */
  if (motor_rx_overflow)
  {
    motor_rx_overflow = 0U;
    MotorCAN_PushError(MOTOR_CAN_ERROR_RX_OVERFLOW);
  }

  /*
   * Bus error callback 只累積旗標；所有停止、事件產生與 FDCAN restart
   * 都在主迴圈處理。這可避免在 ISR 裡進入 HAL 的等待迴圈。
   */
  now = HAL_GetTick();
  for (index = 0U; index < MOTOR_CAN_BUS_COUNT; index++)
  {
    uint32_t error_flags = MotorCAN_TakeBusErrorFlags(index);

    if (error_flags != 0U)
    {
      if ((motor_context.operation != MOTOR_CAN_OPERATION_NONE) &&
          (motor_context.bus == (uint8_t)(index + 1U)))
      {
        MotorCAN_FailOperation(
          MOTOR_CAN_ERROR_BUS,
          MotorCAN_IsMotionOperation());
      }
      else
      {
        MotorCAN_Event event = {0};
        event.type = MOTOR_CAN_EVENT_ERROR;
        event.error = MOTOR_CAN_ERROR_BUS;
        event.bus = (uint8_t)(index + 1U);
        MotorCAN_PushEvent(&event);
      }

      if ((error_flags & MOTOR_CAN_RECOVERY_ERROR_FLAGS) != 0U)
      {
        /* Error passive 或 bus-off 後停止送訊，交由主迴圈安全地重啟該路 FDCAN。 */
        motor_bus_online[index] = 0U;
        motor_bus_recovery_pending[index] = 1U;
        motor_bus_recovery_deadline[index] = now;
      }
    }

    if (motor_bus_recovery_pending[index] &&
        MotorCAN_DeadlineReached(now, motor_bus_recovery_deadline[index]))
    {
      if (MotorCAN_RestartBus((uint8_t)(index + 1U)))
      {
        motor_bus_recovery_pending[index] = 0U;
        motor_bus_recovery_deadline[index] = 0U;
      }
      else
      {
        /* 硬體暫時無法重啟時保留離線狀態，稍後再試，避免主迴圈卡死。 */
        motor_bus_online[index] = 0U;
        motor_bus_recovery_deadline[index] =
          now + MOTOR_CAN_RECOVERY_RETRY_MS;
      }
    }
  }

  /* EMS 優先於 RX frame：同一輪即使收到完成回覆，也先執行緊急停止。 */
  if (MotorCAN_IsMotionOperation() && EMS_IsStopActive())
  {
    MotorCAN_Event event = {0};
    event.type = (motor_context.operation == MOTOR_CAN_OPERATION_HOME) ?
                 MOTOR_CAN_EVENT_HOME_STOPPED_BY_EMS :
                 MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS;
    MotorCAN_StopActiveMotion(1U);
    event.bus = motor_context.bus;
    event.id = motor_context.id;
    MotorCAN_PushEvent(&event);
    MotorCAN_ResetOperation();
  }

  /* ISR 已完成 frame 搬移；此處才做 checksum、command 與 state 判斷。 */
  while (motor_rx_tail != motor_rx_head)
  {
    frame = motor_rx_queue[motor_rx_tail];
    motor_rx_tail = (uint8_t)((motor_rx_tail + 1U) % MOTOR_CAN_RX_QUEUE_SIZE);
    MotorCAN_HandleFrame(&frame);
  }

  MotorCAN_HandleTimeout(HAL_GetTick());
}

/**
  * @brief EMS 釋放時取消 active operation，清除 RX frame 與待回報 event。
  * @retval 無。
  * @note 若取消 TEST/HOME 會緊急停止；取消 PROVISION_1M 會盡力恢復 bus 為 1M。
  */
void MotorCAN_ClearPendingCommands(void)
{
  /*
   * EMS 由 HIGH 回到 LOW 後，主迴圈先呼叫此函式，再解除 command block。
   * 清除 operation、RX frame 與待回報 event，確保 EMS 期間累積的舊狀態
   * 不會在恢復後繼續推進。
   */
  if (!motor_can_ready)
  {
    return;
  }

  /* EMS 若在兩次主迴圈之間快速按下又釋放，清理時仍要送出緊急停止。 */
  if (MotorCAN_IsMotionOperation())
  {
    MotorCAN_StopActiveMotion(1U);
  }

  /* PROVISION_1M 可能暫時把 STM32 CAN 切到 500K，取消時恢復系統速率。 */
  if ((motor_context.operation == MOTOR_CAN_OPERATION_PROVISION_1M) &&
      MotorCAN_IsValidBus(motor_context.bus) &&
      (motor_bus_bitrate_kbps[motor_context.bus - 1U] != MOTOR_CAN_RATE_1M_KBPS))
  {
    (void)MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS);
  }

  MotorCAN_ResetOperation();
  MotorCAN_ClearRxQueue();
  motor_event_tail = motor_event_head;
}

/**
  * @brief 從 event ring buffer 取出最舊的一筆 Motor CAN 結果。
  * @param event 接收事件內容的輸出指標。
  * @retval 1 表示成功取出；0 表示 event 為 NULL 或 queue 為空。
  */
uint8_t MotorCAN_GetEvent(MotorCAN_Event *event)
{
  /* Event queue 只在主迴圈使用，不需關中斷保護。 */
  if ((event == NULL) || (motor_event_tail == motor_event_head))
  {
    return 0U;
  }

  *event = motor_event_queue[motor_event_tail];
  motor_event_tail = (uint8_t)((motor_event_tail + 1U) % MOTOR_CAN_EVENT_QUEUE_SIZE);
  return 1U;
}

/**
  * @brief 查詢目前 active 的高階 Motor CAN 操作。
  * @return INFO、SET_ID、TEST、HOME、PROVISION_1M；閒置時為 NONE。
  */
MotorCAN_Operation MotorCAN_GetOperation(void)
{
  return motor_context.operation;
}

/**
  * @brief HAL FDCAN FIFO0 callback，將硬體 RX frame 搬入軟體 ring buffer。
  * @param hfdcan 觸發 callback 的 FDCAN handle，用來映射邏輯 bus 1/2。
  * @param RxFifo0ITs FIFO0 interrupt bit mask；沒有 NEW_MESSAGE 時直接返回。
  * @retval 無。
  * @note 在 IRQ context 執行，只接受 Standard data frame，不做 checksum 或 state 處理。
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint8_t bus;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  if (hfdcan == motor_can_handles[0])
  {
    bus = 1U;
  }
  else if (hfdcan == motor_can_handles[1])
  {
    bus = 2U;
  }
  else
  {
    return;
  }

  /*
   * 一次中斷把硬體 FIFO0 目前所有 frame 搬到軟體 ring buffer。
   * callback 不驗 checksum、不跑 state machine，也不送回覆，縮短 IRQ latency。
   */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    uint8_t next;
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      break;
    }
    if ((header.IdType != FDCAN_STANDARD_ID) ||
        (header.RxFrameType != FDCAN_DATA_FRAME) ||
        (header.DataLength > FDCAN_DLC_BYTES_8))
    {
      continue;
    }

    next = (uint8_t)((motor_rx_head + 1U) % MOTOR_CAN_RX_QUEUE_SIZE);
    if (next == motor_rx_tail)
    {
      motor_rx_overflow = 1U;
      continue;
    }

    motor_rx_queue[motor_rx_head].bus = bus;
    motor_rx_queue[motor_rx_head].id = (uint16_t)header.Identifier;
    motor_rx_queue[motor_rx_head].length = (uint8_t)header.DataLength;
    memcpy(motor_rx_queue[motor_rx_head].data, data, sizeof(data));
    motor_rx_head = next;
  }
}

/**
  * @brief HAL FDCAN error callback，將錯誤 bit 累積到對應 bus 的共享旗標。
  * @param hfdcan 觸發 callback 的 FDCAN handle。
  * @param ErrorStatusITs HAL 提供的 error-status bit mask。
  * @retval 無。
  * @note 在 IRQ context 執行；真正的事件回報與 bus restart 留給 MotorCAN_Process()。
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  /* 只 OR 累積錯誤旗標；MotorCAN_Process() 會以臨界區取走並執行復原。 */
  if (hfdcan == motor_can_handles[0])
  {
    motor_bus_error_flags[0] |= ErrorStatusITs;
  }
  else if (hfdcan == motor_can_handles[1])
  {
    motor_bus_error_flags[1] |= ErrorStatusITs;
  }
}
