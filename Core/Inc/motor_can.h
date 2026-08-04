#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* 對外操作一次只允許執行一項，避免改 ID 與轉動測試互相干擾。 */
typedef enum
{
  MOTOR_CAN_OPERATION_NONE = 0,
  MOTOR_CAN_OPERATION_INFO,
  MOTOR_CAN_OPERATION_SET_ID,
  MOTOR_CAN_OPERATION_TEST,
  MOTOR_CAN_OPERATION_PROVISION_1M
} MotorCAN_Operation;

typedef enum
{
  MOTOR_CAN_STATUS_OK = 0,
  MOTOR_CAN_STATUS_NOT_READY,
  MOTOR_CAN_STATUS_BUSY,
  MOTOR_CAN_STATUS_INVALID_ARGUMENT,
  MOTOR_CAN_STATUS_EMS_LATCHED,
  MOTOR_CAN_STATUS_TX_FAILED,
  MOTOR_CAN_STATUS_RECONFIG_FAILED
} MotorCAN_Status;

typedef enum
{
  MOTOR_CAN_EVENT_NONE = 0,
  MOTOR_CAN_EVENT_INFO,
  MOTOR_CAN_EVENT_ID_CHANGED,
  MOTOR_CAN_EVENT_MOTOR_RATE_1M,
  MOTOR_CAN_EVENT_TEST_FINISHED,
  MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS,
  MOTOR_CAN_EVENT_ERROR
} MotorCAN_EventType;

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
  MOTOR_CAN_ERROR_RECONFIG_FAILED
} MotorCAN_Error;

typedef struct
{
  MotorCAN_EventType type;
  MotorCAN_Error error;
  uint8_t bus;
  uint16_t id;
  uint16_t old_id;
  uint16_t new_id;
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
  * @note EMS 已鎖存時不會啟動；測試結束後會 Disable 馬達。
  */
MotorCAN_Status MotorCAN_StartTest(uint8_t bus, uint16_t id);

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

/** @brief 取出一筆非同步操作結果。 */
uint8_t MotorCAN_GetEvent(MotorCAN_Event *event);

/** @brief 取得目前執行中的操作。 */
MotorCAN_Operation MotorCAN_GetOperation(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_H */
