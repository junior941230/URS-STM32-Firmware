#include "usbd_cdc_if.h"

#include "usb_command.h"
#include "usb_device.h"

#include <string.h>

/*
 * USB Device CDC class 與專案 command parser 的接合層。
 *
 * OUT endpoint 收到資料時，CDC_Receive_FS() 只轉交給 USB_Command_OnReceive()
 * 並立刻重新 arm 接收。IN endpoint 完成傳送時，CDC_TransmitCplt_FS() 通知
 * command 層釋放 TX queue slot。實際命令解析與回覆排程都在主迴圈執行。
 */

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
static uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern USBD_HandleTypeDef hUsbDeviceFS;

/*
 * CDC ACM 的 line coding 只是主機端 serial port 相容資訊；USB bulk endpoint
 * 不會真的改變 baud rate。仍保存 SET 值，讓後續 GET_LINE_CODING 能原樣回覆。
 */
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

/**
 * @brief 初始化 USB CDC 類別使用的收發緩衝區。
 *
 * USB Device 完成 CDC 類別初始化時會呼叫此函式。它把本檔案配置的
 * `UserTxBufferFS` 與 `UserRxBufferFS` 註冊給 ST 的 CDC class driver，
 * 讓後續 IN/OUT endpoint 傳輸知道要從哪一塊記憶體讀寫資料。
 *
 * @return 固定回傳 `USBD_OK`，表示介面初始化完成。
 * @note 這是 USB middleware callback，不是由主迴圈直接呼叫。
 */
static int8_t CDC_Init_FS(void)
{
  /* 將 class driver 的初始 TX/RX buffer 指向本檔的靜態儲存區。 */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0U);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (int8_t)USBD_OK;
}

/**
 * @brief 解除 USB CDC 介面並同步清除命令層狀態。
 *
 * 主機拔除、USB reset 或 configuration 被取消時，USB middleware 會呼叫
 * 此函式。除了結束 CDC 介面，也通知 `usb_command` 丟棄尚未送完的 TX
 * 訊息，避免重新連線後誤送舊回覆。
 *
 * @return 固定回傳 `USBD_OK`。
 * @note 可能在 USB callback／中斷相關路徑中執行，內部不可阻塞等待。
 */
static int8_t CDC_DeInit_FS(void)
{
  /* 裝置取消 configuration 或斷線時，捨棄沒有機會送完的 command 回覆。 */
  USB_Command_OnDisconnect();
  return (int8_t)USBD_OK;
}

/**
 * @brief 處理主機送給 CDC ACM 的 class-specific control request。
 *
 * 目前保存與回傳 7-byte line coding（baud rate、stop bit、parity、data bit）。
 * 這些設定是為了符合虛擬 COM port 協定；實際資料走 USB bulk endpoint，
 * 不會因主機選擇的 baud rate 而改變傳輸速度。
 *
 * @param command CDC request 代碼，例如 `CDC_SET_LINE_CODING`。
 * @param buffer request 的資料緩衝區；內容方向依 `command` 而定。
 * @param length `buffer` 可用長度，複製 line coding 前會先檢查是否至少 7 bytes。
 * @return 固定回傳 `USBD_OK`；目前未支援的 request 會安全忽略。
 */
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

/**
 * @brief 接收 USB OUT endpoint 資料並轉交文字命令解析層。
 *
 * 先把本批資料交給 `USB_Command_OnReceive()` 寫入 RX ring buffer，再重新
 * 設定接收緩衝區並 arm 下一次 OUT transfer。即使收到空指標，也仍會重新
 * 開啟接收，避免 endpoint 因單次異常而永久停止。
 *
 * @param buffer 本次由主機送入的資料起點，可為 `NULL`。
 * @param length 指向本次有效資料長度，可為 `NULL`。
 * @return 固定回傳 `USBD_OK`。
 * @note 此 callback 位於 USB 接收路徑，只搬移資料，不在此解析或執行命令。
 */
static int8_t CDC_Receive_FS(uint8_t *buffer, uint32_t *length)
{
  if ((buffer != NULL) && (length != NULL))
  {
    USB_Command_OnReceive(buffer, *length);
  }

  /* 無論 command 層是否接受資料，都要重新 arm OUT endpoint，否則主機會停收。 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

/**
 * @brief 嘗試透過 CDC IN endpoint 非同步送出一段資料。
 *
 * 函式會先確認參數、USB configured 狀態與 CDC class handle，再檢查上一筆
 * IN transfer 是否完成。成功排入傳輸只代表 middleware 已接受資料；真正
 * 傳完後會由 `CDC_TransmitCplt_FS()` 通知命令層釋放對應 TX queue slot。
 *
 * @param buffer 要送出的資料；傳輸完成前內容必須保持有效。
 * @param length 要送出的 byte 數，不可為 0。
 * @return `USBD_OK` 表示已開始傳輸；`USBD_BUSY` 表示前一筆仍在傳送；
 *         `USBD_FAIL` 表示參數無效、USB 尚未 configured 或 class 尚未就緒。
 */
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length)
{
  USBD_CDC_HandleTypeDef *cdc_handle;

  /* USB 尚未 configured 或 class data 尚未配置時，不可解參考 CDC handle。 */
  if ((buffer == NULL) || (length == 0U) ||
      (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) ||
      (hUsbDeviceFS.pClassData == NULL))
  {
    return USBD_FAIL;
  }

  cdc_handle = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  /* 一次只允許一筆 IN transfer；command 層遇到 BUSY 會在下一輪重試同一 slot。 */
  if (cdc_handle->TxState != 0U)
  {
    return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, buffer, length);
  return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/**
 * @brief 處理 CDC IN transfer 完成事件。
 *
 * ST middleware 在 USB peripheral 已不再使用傳送緩衝區後呼叫此函式。
 * 命令層收到通知後才會推進 TX ring buffer 的 tail，確保非同步傳輸期間
 * 該 slot 不會被覆寫。
 *
 * @param buffer middleware 回傳的傳送緩衝區；本實作不需使用。
 * @param length middleware 回傳的傳送長度；本實作不需使用。
 * @param endpoint 完成傳輸的 endpoint 編號；本實作不需使用。
 * @return 固定回傳 `USBD_OK`。
 * @note 此 callback 位於 USB 傳輸完成路徑，不可執行阻塞操作。
 */
static int8_t CDC_TransmitCplt_FS(uint8_t *buffer, uint32_t *length,
                                 uint8_t endpoint)
{
  (void)buffer;
  (void)length;
  (void)endpoint;
  /* 到這裡 USB peripheral 已不再使用原 buffer，command 層才能移動 TX tail。 */
  USB_Command_OnTransmitComplete();
  return (int8_t)USBD_OK;
}
