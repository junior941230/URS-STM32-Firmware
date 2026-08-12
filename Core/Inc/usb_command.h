#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief 初始化 USB CDC 文字命令 parser 與收發 queue。
  * @note 必須在 MX_USB_Device_Init() 前呼叫，避免 USB 列舉期間讀到舊狀態。
  */
void USB_Command_Init(void);

/**
  * @brief 在主迴圈解析命令、處理 CAN 結果並送出回覆。
  *
  * 函式不會阻塞等待 CAN 或 USB；每次只推進目前可處理的 queue 與狀態。
  */
void USB_Command_Process(void);

/**
  * @brief EMS 釋放時清除尚未解析及尚未完成的輸入指令。
  *
  * 這能避免 EMS 啟動期間已送入、但尚未執行的命令在釋放後突然生效。
  */
void USB_Command_ClearPendingCommands(void);

/**
  * @brief 由 USB CDC receive callback 將原始 bytes 放入 RX ring buffer。
  * @note callback context 只搬資料；文字解析留給 USB_Command_Process()。
  */
void USB_Command_OnReceive(const uint8_t *data, uint32_t length);

/** @brief 由 USB CDC transmit complete callback 通知 TX queue 釋放目前封包。 */
void USB_Command_OnTransmitComplete(void);

/** @brief USB 中斷連線時清除尚未完成的傳送 queue 與 inflight 狀態。 */
void USB_Command_OnDisconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMMAND_H */
