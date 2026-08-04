#include "usbd_cdc_if.h"

#include "usb_command.h"
#include "usb_device.h"

#include <string.h>

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern USBD_HandleTypeDef hUsbDeviceFS;

/* 主機設定的 line coding 不影響 USB bulk 傳輸，仍保留供 CDC ACM 相容。 */
static uint8_t line_coding[7] = {
  0x00U, 0xC2U, 0x01U, 0x00U, /* 115200 bit/s */
  0x00U,                       /* 1 stop bit */
  0x00U,                       /* no parity */
  0x08U                        /* 8 data bits */
};

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length);
static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length,
                                 uint8_t endpoint);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
  USB_Command_OnDisconnect();
  return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t command, uint8_t *buffer, uint16_t length)
{
  switch (command)
  {
    case CDC_SET_LINE_CODING:
      if ((buffer != NULL) && (length >= sizeof(line_coding)))
      {
        memcpy(line_coding, buffer, sizeof(line_coding));
      }
      break;

    case CDC_GET_LINE_CODING:
      if ((buffer != NULL) && (length >= sizeof(line_coding)))
      {
        memcpy(buffer, line_coding, sizeof(line_coding));
      }
      break;

    case CDC_SET_CONTROL_LINE_STATE:
    default:
      break;
  }

  return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
  if ((buffer != NULL) && (length != NULL))
  {
    USB_Command_OnReceive(buffer, *length);
  }

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
  USBD_CDC_HandleTypeDef *cdc_handle;

  if ((buffer == NULL) || (length == 0U) ||
      (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL))
  {
    return USBD_FAIL;
  }

  cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (cdc_handle->TxState != 0U)
  {
    return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length,
                                 uint8_t endpoint)
{
  (void)buffer;
  (void)length;
  (void)endpoint;
  USB_Command_OnTransmitComplete();
  return (int8_t)USBD_OK;
}
