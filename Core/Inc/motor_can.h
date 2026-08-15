#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/**
  * @brief Motor CAN 模組目前執行中的高階操作。
  *
  * 模組使用單一非同步 state machine，所以同一時間只允許一項操作。
  * 呼叫任一 MotorCAN_Start*() 後，應持續在主迴圈呼叫
  * MotorCAN_Process()，直到 MotorCAN_GetEvent() 取到完成或錯誤事件。
  */
typedef enum
{
  MOTOR_CAN_OPERATION_NONE = 0,
  MOTOR_CAN_OPERATION_INFO,
  MOTOR_CAN_OPERATION_SET_ID,
  MOTOR_CAN_OPERATION_TEST,
  MOTOR_CAN_OPERATION_HOME,
  MOTOR_CAN_OPERATION_INIT,
  MOTOR_CAN_OPERATION_ROTATE,
  MOTOR_CAN_OPERATION_PROVISION_1M
} MotorCAN_Operation;

/**
  * @brief 啟動操作時立即回傳的狀態。
  *
  * 這些值只表示「是否成功接受要求」。真正的馬達執行結果會稍後透過
  * MotorCAN_Event 回報。
  */
typedef enum
{
  MOTOR_CAN_STATUS_OK = 0,
  MOTOR_CAN_STATUS_NOT_READY,
  MOTOR_CAN_STATUS_BUSY,
  MOTOR_CAN_STATUS_INVALID_ARGUMENT,
  MOTOR_CAN_STATUS_EMS_ACTIVE,
  MOTOR_CAN_STATUS_INIT_REQUIRED,
  MOTOR_CAN_STATUS_TX_FAILED,
  MOTOR_CAN_STATUS_RECONFIG_FAILED
} MotorCAN_Status;

/** @brief 非同步操作完成後，由 event queue 回報給 USB command 層的事件。 */
typedef enum
{
  MOTOR_CAN_EVENT_NONE = 0,
  MOTOR_CAN_EVENT_INFO,
  MOTOR_CAN_EVENT_ID_CHANGED,
  MOTOR_CAN_EVENT_MOTOR_RATE_1M,
  MOTOR_CAN_EVENT_TEST_FINISHED,
  MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS,
  MOTOR_CAN_EVENT_HOME_FINISHED,
  MOTOR_CAN_EVENT_HOME_STOPPED_BY_EMS,
  MOTOR_CAN_EVENT_INIT_BIG_PROGRESS,
  MOTOR_CAN_EVENT_INIT_FINISHED,
  MOTOR_CAN_EVENT_INIT_STOPPED_BY_EMS,
  MOTOR_CAN_EVENT_ROTATE_FINISHED,
  MOTOR_CAN_EVENT_ROTATE_STOPPED_BY_EMS,
  MOTOR_CAN_EVENT_ERROR
} MotorCAN_EventType;

/** @brief Motor CAN state machine 對外回報的失敗原因。 */
typedef enum
{
  MOTOR_CAN_ERROR_NONE = 0,
  MOTOR_CAN_ERROR_TARGET_NOT_FOUND,
  MOTOR_CAN_ERROR_NEW_ID_IN_USE,
  MOTOR_CAN_ERROR_DEVICE_REJECTED,
  MOTOR_CAN_ERROR_SAVE_FAILED,
  MOTOR_CAN_ERROR_VERIFY_FAILED,
  MOTOR_CAN_ERROR_RESPONSE_TIMEOUT,
  MOTOR_CAN_ERROR_RX_OVERFLOW,
  MOTOR_CAN_ERROR_BUS,
  MOTOR_CAN_ERROR_TX,
  MOTOR_CAN_ERROR_RECONFIG_FAILED,
  MOTOR_CAN_ERROR_HOME_TIMEOUT
} MotorCAN_Error;

/**
  * @brief 一筆非同步 Motor CAN 結果。
  *
  * 不同事件只會使用其中一部分欄位。例如 INFO 會填入版本欄位，SET_ID
  * 會填入 old_id/new_id，ERROR 會填入 error；未使用欄位維持 0。
  */
typedef struct
{
  MotorCAN_EventType type;
  MotorCAN_Error error;
  MotorCAN_Operation operation;
  uint8_t bus;
  uint16_t id;
  uint16_t old_id;
  uint16_t new_id;
  uint8_t completed_count;
  uint16_t completed_mask;
  uint16_t missing_mask;
  uint8_t hardware_version;
  uint8_t firmware_version[3];
} MotorCAN_Event;

/**
  * @brief 初始化兩條邏輯 CAN 匯流排並開始接收 MKS 回覆
  * @param can1_handle 邏輯 CAN1，硬體對應 FDCAN3
  * @param can2_handle 邏輯 CAN2，硬體對應 FDCAN2
  * @retval MOTOR_CAN_STATUS_OK 表示兩條匯流排均已啟動
  */
MotorCAN_Status MotorCAN_Init(FDCAN_HandleTypeDef *can1_handle,
                              FDCAN_HandleTypeDef *can2_handle);

/** @brief 在主迴圈處理 CAN 回覆、逾時與 EMS 停止要求。 */
void MotorCAN_Process(void);

/**
  * @brief EMS 釋放時停止並清除所有尚未完成的馬達指令。
  * @retval 1 表示兩條 bus 都已排入全域停止與 Disable；0 表示下輪需重試。
  * @note 若正在 PROVISION_1M，會盡力把 STM32 CAN bus 恢復為 1 Mbit/s。
  */
uint8_t MotorCAN_ClearPendingCommands(void);

/** @brief 讀取指定 MKS Servo42ES 的硬體與韌體版本。 */
MotorCAN_Status MotorCAN_StartInfo(uint8_t bus, uint16_t id);

/**
  * @brief 修改單顆 MKS Servo42ES 的 CAN ID 並寫入內部儲存
  * @note 同一匯流排若有多顆馬達使用 old_id，必須先只保留目標馬達上線。
  */
MotorCAN_Status MotorCAN_StartSetId(uint8_t bus, uint16_t old_id,
                                    uint16_t new_id);

/**
  * @brief 執行固定 30 RPM、500 ms 的低速短時間測試
  * @note EMS 啟動時不會執行；測試結束後會 Disable 馬達。
  */
MotorCAN_Status MotorCAN_StartTest(uint8_t bus, uint16_t id);

/**
  * @brief 使用 Servo42ES 的 origin switch 執行 homing
  * @param direction 0 代表正向，1 代表反向
  * @param high_speed_rpm 尋找原點的高速段，範圍 1..3000 RPM
  * @param low_speed_rpm 精確尋找原點的低速段，範圍 1..100 RPM
  * @param offset_angle_degrees 找到開關後的 signed offset 角度
  * @param timeout_ms 馬達內部 homing timeout，範圍 1000..120000 ms
  * @note PCB1 的 RPI-352 經 2N7002 接到馬達 IN-，因此使用 active-low 原點開關設定。
  */
MotorCAN_Status MotorCAN_StartHome(uint8_t bus, uint16_t id,
                                   uint8_t direction,
                                   uint16_t high_speed_rpm,
                                   uint16_t low_speed_rpm,
                                   double offset_angle_degrees,
                                   uint32_t timeout_ms);

/**
  * @brief 先同步 homing 所有 big motor，再依序 homing 所有 small motor。
  * @note 馬達清單取自 MachineState；成功前 TEST、HOME、ROTATE 都會被拒絕。
  */
MotorCAN_Status MotorCAN_StartInit(void);

/**
  * @brief 使用各自的 signed offset 角度執行 big/small motor 初始化。
  * @note 負角度會先以 offset 0 homing；big offset 完成後才進入 small homing，
  *       各組 rotate 完都會設為新零點。
  */
MotorCAN_Status
MotorCAN_StartInitWithOffsetAngles(double big_offset_angle_degrees,
                                   double small_offset_angle_degrees);

/** @brief 回傳本次開機是否已完整執行 INIT。 */
uint8_t MotorCAN_IsInitialized(void);

/**
  * @brief 依 machine_state 規劃並同步啟動一組 Rubik 馬達動作。
  * @param bus face 尚未設定時使用的 1-based CAN bus。
  * @param id face 尚未設定時使用的主要馬達 ID。
  * @param command R、R_、Rw、Rw_ 等 Rubik command。
  * @note 會使用 Servo42D 4AH Synchronization mark 與 broadcast 4BH 同步啟動。
  */
MotorCAN_Status MotorCAN_StartRotate(uint8_t bus, uint16_t id,
                                     const char *command);

/**
  * @brief 將指定 STM32 CAN bus 切換為 500 或 1000 kbit/s。
  * @note 只能在沒有進行中的馬達操作時切換。
  */
MotorCAN_Status MotorCAN_SetBusBitrate(uint8_t bus, uint16_t bitrate_kbps);

/** @brief 讀取指定 STM32 CAN bus 目前使用的 bitrate，無效 bus 回傳 0。 */
uint16_t MotorCAN_GetBusBitrate(uint8_t bus);

/**
  * @brief 將單顆出廠 500 kbit/s 馬達改為 1 Mbit/s、儲存並驗證。
  * @note 流程結束或失敗時，會盡力把指定 STM32 CAN bus 恢復為 1 Mbit/s。
  */
MotorCAN_Status MotorCAN_StartProvision1M(uint8_t bus, uint16_t id);

/**
  * @brief 取出一筆非同步操作結果。
  * @retval 1 已將事件寫入 event；0 表示 queue 為空或 event 是 NULL。
  */
uint8_t MotorCAN_GetEvent(MotorCAN_Event *event);

/** @brief 取得目前執行中的操作；閒置時回傳 MOTOR_CAN_OPERATION_NONE。 */
MotorCAN_Operation MotorCAN_GetOperation(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_H */
