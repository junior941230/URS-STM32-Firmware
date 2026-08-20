#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/**
  * @brief 初始化 NUCLEO ST-LINK VCP 使用的 LPUART1 文字命令收發。
  * @param uart 已由 BSP_COM_Init() 設成 115200 8N1 的 COM1 handle。
  * @retval HAL_OK 表示 RX interrupt 已啟動；其他值表示初始化失敗。
  */
HAL_StatusTypeDef Serial_Command_Init(UART_HandleTypeDef *uart);

/**
  * @brief 在主迴圈解析命令、處理 CAN 結果並送出回覆。
  *
  * 函式不會阻塞等待 CAN 或 UART；每次只推進目前可處理的 queue 與狀態。
  */
void Serial_Command_Process(void);

/**
  * @brief EMS 釋放時清除尚未解析及尚未完成的輸入指令。
  *
  * 這能避免 EMS 啟動期間已送入、但尚未執行的命令在釋放後突然生效。
  */
void Serial_Command_ClearPendingCommands(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COMMAND_H */
