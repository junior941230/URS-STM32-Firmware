#include "motor_can.h"

#include "main.h"

#include <string.h>

#define MOTOR_CAN_BUS_COUNT             2U
#define MOTOR_CAN_MAX_STANDARD_ID       0x7FFU
#define MOTOR_CAN_RX_QUEUE_SIZE         16U
#define MOTOR_CAN_EVENT_QUEUE_SIZE      8U
#define MOTOR_CAN_PROBE_TIMEOUT_MS      250U
#define MOTOR_CAN_COMMAND_TIMEOUT_MS    800U
#define MOTOR_CAN_VERIFY_DELAY_MS       200U
#define MOTOR_CAN_RESTART_DELAY_MS      1000U
#define MOTOR_CAN_TEST_TIMEOUT_MS       2000U
#define MOTOR_CAN_TEST_SPEED_RPM        30U
#define MOTOR_CAN_TEST_ACCELERATION     2U
#define MOTOR_CAN_TEST_RUNTIME_10MS     50U
#define MOTOR_CAN_RATE_500_KBPS         500U
#define MOTOR_CAN_RATE_1M_KBPS          1000U
#define MOTOR_CAN_PRESCALER_500K        20U
#define MOTOR_CAN_PRESCALER_1M          10U
#define MOTOR_CAN_RECOVERY_RETRY_MS      250U
#define MOTOR_CAN_NOTIFICATIONS         (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                                         FDCAN_IT_BUS_OFF | \
                                         FDCAN_IT_ERROR_PASSIVE | \
                                         FDCAN_IT_ERROR_WARNING)
#define MOTOR_CAN_RECOVERY_ERROR_FLAGS  (FDCAN_IT_BUS_OFF | \
                                         FDCAN_IT_ERROR_PASSIVE)

typedef struct
{
  uint8_t bus;
  uint16_t id;
  uint8_t length;
  uint8_t data[8];
} MotorCAN_RxFrame;

typedef enum
{
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_INFO_WAIT,
  MOTOR_STATE_SET_PROBE_OLD,
  MOTOR_STATE_SET_PROBE_NEW,
  MOTOR_STATE_SET_WAIT_ACK,
  MOTOR_STATE_SET_FIND_NEW,
  MOTOR_STATE_SET_FIND_OLD,
  MOTOR_STATE_SET_WAIT_SAVE,
  MOTOR_STATE_SET_WAIT_VERIFY_DELAY,
  MOTOR_STATE_SET_WAIT_VERIFY,
  MOTOR_STATE_TEST_PROBE,
  MOTOR_STATE_TEST_WAIT_ENABLE,
  MOTOR_STATE_TEST_WAIT_RUN,
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
  MotorCAN_Operation operation;
  MotorCAN_State state;
  uint8_t bus;
  uint16_t id;
  uint16_t old_id;
  uint16_t new_id;
  uint16_t active_id;
  uint8_t set_ack_received;
  uint8_t rate_save_ack_received;
  uint32_t deadline;
} MotorCAN_Context;

static FDCAN_HandleTypeDef *motor_can_handles[MOTOR_CAN_BUS_COUNT];
static uint16_t motor_bus_bitrate_kbps[MOTOR_CAN_BUS_COUNT];
static uint8_t motor_bus_online[MOTOR_CAN_BUS_COUNT];
static MotorCAN_Context motor_context;
static uint8_t motor_can_ready;

/* RX queue 由 FDCAN ISR 寫入、主迴圈讀出。 */
static MotorCAN_RxFrame motor_rx_queue[MOTOR_CAN_RX_QUEUE_SIZE];
static volatile uint8_t motor_rx_head;
static volatile uint8_t motor_rx_tail;
static volatile uint8_t motor_rx_overflow;
static volatile uint32_t motor_bus_error_flags[MOTOR_CAN_BUS_COUNT];
static uint8_t motor_bus_recovery_pending[MOTOR_CAN_BUS_COUNT];
static uint32_t motor_bus_recovery_deadline[MOTOR_CAN_BUS_COUNT];

/* Event queue 只在主迴圈寫入，USB command 模組讀出。 */
static MotorCAN_Event motor_event_queue[MOTOR_CAN_EVENT_QUEUE_SIZE];
static uint8_t motor_event_head;
static uint8_t motor_event_tail;

static uint8_t MotorCAN_IsValidBus(uint8_t bus)
{
  return (bus >= 1U) && (bus <= MOTOR_CAN_BUS_COUNT);
}

static uint8_t MotorCAN_IsValidNodeId(uint16_t id)
{
  /* ID 0 是 broadcast；改設定與測試一律禁止使用。 */
  return (id >= 1U) && (id <= MOTOR_CAN_MAX_STANDARD_ID);
}

static uint8_t MotorCAN_DeadlineReached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t MotorCAN_IsSupportedBitrate(uint16_t bitrate_kbps)
{
  return (bitrate_kbps == MOTOR_CAN_RATE_500_KBPS) ||
         (bitrate_kbps == MOTOR_CAN_RATE_1M_KBPS);
}

static void MotorCAN_ClearRxQueue(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  motor_rx_tail = motor_rx_head;
  motor_rx_overflow = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

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

  /* 切換 bitrate 前先停用中斷並重建 FDCAN，避免殘留舊速率的 frame。 */
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

static void MotorCAN_ResetOperation(void)
{
  memset(&motor_context, 0, sizeof(motor_context));
  motor_context.operation = MOTOR_CAN_OPERATION_NONE;
  motor_context.state = MOTOR_STATE_IDLE;
}

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

static uint8_t MotorCAN_SendReadVersion(uint8_t bus, uint16_t id)
{
  const uint8_t body[] = {0x40U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

static uint8_t MotorCAN_SendSetId(uint8_t bus, uint16_t old_id, uint16_t new_id)
{
  const uint8_t body[] = {
    0x8BU,
    (uint8_t)((new_id >> 8) & 0xFFU),
    (uint8_t)(new_id & 0xFFU)
  };
  return MotorCAN_SendBody(bus, old_id, body, sizeof(body));
}

static uint8_t MotorCAN_SendSetBitrate1M(uint8_t bus, uint16_t id)
{
  /* MKS 手冊：0x8A 的參數 0x03 代表 1 Mbit/s。 */
  const uint8_t body[] = {0x8AU, 0x03U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

static uint8_t MotorCAN_SendSave(uint8_t bus, uint16_t id)
{
  const uint8_t body[] = {0x60U, 0x01U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

static uint8_t MotorCAN_SendReset(uint8_t bus, uint16_t id)
{
  /* MKS 手冊：0x41 只重啟控制器，不會清除已儲存的參數。 */
  const uint8_t body[] = {0x41U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

static uint8_t MotorCAN_SendEnable(uint8_t bus, uint16_t id, uint8_t enable)
{
  const uint8_t body[] = {0xF3U, enable ? 0x01U : 0x00U};
  return MotorCAN_SendBody(bus, id, body, sizeof(body));
}

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

static void MotorCAN_StopKnownLowSpeedTest(uint8_t emergency)
{
  if ((motor_context.operation != MOTOR_CAN_OPERATION_TEST) ||
      (!MotorCAN_IsValidNodeId(motor_context.id)))
  {
    return;
  }

  if (emergency)
  {
    /* 本模組只會以 30 RPM 測試，EMS 時使用手冊 0xF7 控制停止。 */
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

static uint8_t MotorCAN_FrameChecksumIsValid(const MotorCAN_RxFrame *frame)
{
  uint16_t sum;
  uint8_t index;

  if ((frame == NULL) || (frame->length < 2U) || (frame->length > 8U))
  {
    return 0U;
  }

  sum = frame->id;
  for (index = 0U; index < (uint8_t)(frame->length - 1U); index++)
  {
    sum = (uint16_t)(sum + frame->data[index]);
  }

  return (((uint8_t)(sum & 0xFFU)) == frame->data[frame->length - 1U]) ? 1U : 0U;
}

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

static void MotorCAN_CompleteProvision1M(void)
{
  MotorCAN_Event event = {0};

  event.type = MOTOR_CAN_EVENT_MOTOR_RATE_1M;
  event.bus = motor_context.bus;
  event.id = motor_context.id;
  MotorCAN_PushEvent(&event);
  MotorCAN_ResetOperation();
}

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

static void MotorCAN_FailOperation(MotorCAN_Error error, uint8_t stop_test)
{
  if (stop_test)
  {
    MotorCAN_StopKnownLowSpeedTest(0U);
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

static void MotorCAN_SendOrFail(uint8_t sent, MotorCAN_State next_state,
                                uint32_t timeout_ms)
{
  if (!sent)
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_TX,
                           motor_context.operation == MOTOR_CAN_OPERATION_TEST);
    return;
  }

  motor_context.state = next_state;
  motor_context.deadline = HAL_GetTick() + timeout_ms;
}

static void MotorCAN_SwitchProvisionTo1MAndProbe(void)
{
  if (!MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS))
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_RECONFIG_FAILED, 0U);
    return;
  }

  MotorCAN_SendOrFail(
    MotorCAN_SendReadVersion(motor_context.bus, motor_context.id),
    MOTOR_STATE_RATE_FIND_1M, MOTOR_CAN_PROBE_TIMEOUT_MS);
}

static void MotorCAN_SwitchProvisionTo1MAndWaitForRestart(void)
{
  if (!MotorCAN_ReconfigureBus(motor_context.bus, MOTOR_CAN_RATE_1M_KBPS))
  {
    MotorCAN_FailOperation(MOTOR_CAN_ERROR_RECONFIG_FAILED, 0U);
    return;
  }

  motor_context.state = MOTOR_STATE_RATE_WAIT_REBOOT_1M;
  motor_context.deadline = HAL_GetTick() + MOTOR_CAN_RESTART_DELAY_MS;
}

static void MotorCAN_HandleFrame(const MotorCAN_RxFrame *frame)
{
  if ((!MotorCAN_FrameChecksumIsValid(frame)) ||
      (frame->bus != motor_context.bus) ||
      (motor_context.operation == MOTOR_CAN_OPERATION_NONE))
  {
    return;
  }

  switch (motor_context.state)
  {
    case MOTOR_STATE_INFO_WAIT:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteInfo(frame);
      }
      break;

    case MOTOR_STATE_SET_PROBE_OLD:
      if ((frame->id == motor_context.old_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendReadVersion(motor_context.bus, motor_context.new_id),
          MOTOR_STATE_SET_PROBE_NEW, MOTOR_CAN_PROBE_TIMEOUT_MS);
      }
      break;

    case MOTOR_STATE_SET_PROBE_NEW:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_FailOperation(MOTOR_CAN_ERROR_NEW_ID_IN_USE, 0U);
      }
      break;

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

    case MOTOR_STATE_SET_FIND_NEW:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        motor_context.active_id = motor_context.new_id;
        MotorCAN_SendOrFail(
          MotorCAN_SendSave(motor_context.bus, motor_context.active_id),
          MOTOR_STATE_SET_WAIT_SAVE, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

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

    case MOTOR_STATE_SET_WAIT_VERIFY:
      if ((frame->id == motor_context.new_id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteIdChange();
      }
      break;

    case MOTOR_STATE_RATE_PROBE_500:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_SendOrFail(
          MotorCAN_SendSetBitrate1M(motor_context.bus, motor_context.id),
          MOTOR_STATE_RATE_WAIT_SET_ACK_500, MOTOR_CAN_COMMAND_TIMEOUT_MS);
      }
      break;

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

    case MOTOR_STATE_RATE_WAIT_VERIFY:
      if ((frame->id == motor_context.id) && (frame->data[0] == 0x40U))
      {
        MotorCAN_CompleteProvision1M();
      }
      break;

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

    default:
      break;
  }
}

static void MotorCAN_HandleTimeout(uint32_t now)
{
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
    case MOTOR_STATE_RATE_PROBE_500:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_TARGET_NOT_FOUND,
                             motor_context.operation == MOTOR_CAN_OPERATION_TEST);
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

    default:
      MotorCAN_FailOperation(MOTOR_CAN_ERROR_RESPONSE_TIMEOUT,
                             motor_context.operation == MOTOR_CAN_OPERATION_TEST);
      break;
  }
}

MotorCAN_Status MotorCAN_Init(FDCAN_HandleTypeDef *can1_handle,
                              FDCAN_HandleTypeDef *can2_handle)
{
  uint8_t index;

  if ((can1_handle == NULL) || (can2_handle == NULL))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }

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
    /* 接收所有 Standard data frame；拒絕 Extended 與 Remote frame。 */
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

uint16_t MotorCAN_GetBusBitrate(uint8_t bus)
{
  if ((!MotorCAN_IsValidBus(bus)) || (!motor_bus_online[bus - 1U]))
  {
    return 0U;
  }
  return motor_bus_bitrate_kbps[bus - 1U];
}

MotorCAN_Status MotorCAN_StartProvision1M(uint8_t bus, uint16_t id)
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

MotorCAN_Status MotorCAN_StartSetId(uint8_t bus, uint16_t old_id,
                                    uint16_t new_id)
{
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

MotorCAN_Status MotorCAN_StartTest(uint8_t bus, uint16_t id)
{
  if (!motor_can_ready)
  {
    return MOTOR_CAN_STATUS_NOT_READY;
  }
  if ((!MotorCAN_IsValidBus(bus)) || (!MotorCAN_IsValidNodeId(id)))
  {
    return MOTOR_CAN_STATUS_INVALID_ARGUMENT;
  }
  if (EMS_IsStopLatched())
  {
    return MOTOR_CAN_STATUS_EMS_LATCHED;
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

void MotorCAN_Process(void)
{
  MotorCAN_RxFrame frame;
  uint32_t now;
  uint8_t index;

  if (!motor_can_ready)
  {
    return;
  }

  if (motor_rx_overflow)
  {
    motor_rx_overflow = 0U;
    MotorCAN_PushError(MOTOR_CAN_ERROR_RX_OVERFLOW);
  }

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
          motor_context.operation == MOTOR_CAN_OPERATION_TEST);
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

  if ((motor_context.operation == MOTOR_CAN_OPERATION_TEST) &&
      EMS_IsStopLatched())
  {
    MotorCAN_Event event = {0};
    MotorCAN_StopKnownLowSpeedTest(1U);
    event.type = MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS;
    event.bus = motor_context.bus;
    event.id = motor_context.id;
    MotorCAN_PushEvent(&event);
    MotorCAN_ResetOperation();
  }

  while (motor_rx_tail != motor_rx_head)
  {
    frame = motor_rx_queue[motor_rx_tail];
    motor_rx_tail = (uint8_t)((motor_rx_tail + 1U) % MOTOR_CAN_RX_QUEUE_SIZE);
    MotorCAN_HandleFrame(&frame);
  }

  MotorCAN_HandleTimeout(HAL_GetTick());
}

uint8_t MotorCAN_GetEvent(MotorCAN_Event *event)
{
  if ((event == NULL) || (motor_event_tail == motor_event_head))
  {
    return 0U;
  }

  *event = motor_event_queue[motor_event_tail];
  motor_event_tail = (uint8_t)((motor_event_tail + 1U) % MOTOR_CAN_EVENT_QUEUE_SIZE);
  return 1U;
}

MotorCAN_Operation MotorCAN_GetOperation(void)
{
  return motor_context.operation;
}

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

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  if (hfdcan == motor_can_handles[0])
  {
    motor_bus_error_flags[0] |= ErrorStatusITs;
  }
  else if (hfdcan == motor_can_handles[1])
  {
    motor_bus_error_flags[1] |= ErrorStatusITs;
  }
}
