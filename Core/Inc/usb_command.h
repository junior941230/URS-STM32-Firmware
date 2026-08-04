#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 初始化 USB CDC 文字命令 parser 與收發 queue。 */
void USB_Command_Init(void);

/** @brief 在主迴圈解析命令、處理 CAN 結果並送出回覆。 */
void USB_Command_Process(void);

/** @brief 由 USB CDC receive callback 放入接收到的資料。 */
void USB_Command_OnReceive(const uint8_t *data, uint32_t length);

/** @brief 由 USB CDC transmit complete callback 通知 queue 釋放封包。 */
void USB_Command_OnTransmitComplete(void);

/** @brief USB 中斷連線時清除尚未完成的傳送狀態。 */
void USB_Command_OnDisconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_COMMAND_H */
