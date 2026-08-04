#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

/* CDC buffers 只承接 USB packet；完整文字命令由 usb_command.c 處理。 */
#define APP_RX_DATA_SIZE  256U
#define APP_TX_DATA_SIZE  256U

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/** @brief 透過 USB CDC 傳送一個 buffer；完成前 buffer 必須保持有效。 */
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
