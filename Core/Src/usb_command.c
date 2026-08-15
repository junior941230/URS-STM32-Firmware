#include "usb_command.h"

#include "main.h"
#include "motor_can.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * USB CDC 文字命令層
 * -----------------
 * CDC receive callback 只把 bytes 放入 RX ring buffer；主迴圈再組成一行、
 * 驗證參數並呼叫 MotorCAN_Start*()。Motor CAN 的完成事件會轉成一行文字，
 * 放入 TX queue，最後由 CDC transmit-complete callback 釋放 slot。
 *
 * 所有回覆以 CRLF 結尾。命令不分大小寫，bus 與 CAN ID 可用十進位，
 * CAN ID 也可用 0x 前綴的十六進位表示。
 */

/* RX 是 byte stream；line buffer 是一條完整命令；TX 每個 slot 保存一整行回覆。
 */
#define USB_COMMAND_RX_QUEUE_SIZE 256U
#define USB_COMMAND_LINE_SIZE 96U
#define USB_COMMAND_TX_QUEUE_SIZE 8U
#define USB_COMMAND_TX_MESSAGE_SIZE 192U

/*
 * RX queue 是 single-producer/single-consumer ring buffer：USB callback 寫
 * head， 主迴圈讀 tail。滿載時設 overflow，主迴圈稍後回報錯誤而不阻塞
 * callback。
 */
static uint8_t usb_rx_queue[USB_COMMAND_RX_QUEUE_SIZE];
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static volatile uint8_t usb_rx_overflow;

/*
 * TX queue 由主迴圈寫入；USB transmit-complete callback 移動 tail。
 * usb_tx_inflight=1 代表 tail 指向的 slot 仍由 USB peripheral 使用，不可覆寫。
 */
static char usb_tx_queue[USB_COMMAND_TX_QUEUE_SIZE]
                        [USB_COMMAND_TX_MESSAGE_SIZE];
static volatile uint8_t usb_tx_head;
static volatile uint8_t usb_tx_tail;
static volatile uint8_t usb_tx_inflight;

static char usb_line[USB_COMMAND_LINE_SIZE];
static uint16_t usb_line_length;
static uint8_t usb_line_overflow;

/**
 * @brief 格式化一行文字、補上 CRLF，並放入 USB TX ring buffer。
 * @param format printf-style 格式字串，後面可帶對應 variadic arguments。
 * @retval 無。
 * @note queue 滿時直接保留既有回覆並捨棄新訊息，不阻塞主迴圈。
 */
static void USB_Command_QueueText(const char *format, ...) {
  uint8_t next;
  int length;
  va_list arguments;

  /* ring buffer 保留一格；next==tail 代表 queue 已滿。 */
  next = (uint8_t)((usb_tx_head + 1U) % USB_COMMAND_TX_QUEUE_SIZE);
  if (next == usb_tx_tail) {
    /* USB host 未讀取時不阻塞主迴圈，保留既有回覆。 */
    return;
  }

  va_start(arguments, format);
  length = vsnprintf(usb_tx_queue[usb_tx_head],
                     USB_COMMAND_TX_MESSAGE_SIZE - 3U, format, arguments);
  va_end(arguments);

  if (length < 0) {
    return;
  }
  if (length > (int)(USB_COMMAND_TX_MESSAGE_SIZE - 4U)) {
    /* vsnprintf 已在此位置補 NUL；改寫成 CRLF 後仍保留結尾 NUL。 */
    length = (int)(USB_COMMAND_TX_MESSAGE_SIZE - 4U);
  }

  usb_tx_queue[usb_tx_head][length++] = '\r';
  usb_tx_queue[usb_tx_head][length++] = '\n';
  usb_tx_queue[usb_tx_head][length] = '\0';
  usb_tx_head = next;
}

/**
 * @brief 把 MotorCAN_Operation 轉成 STATUS 回覆使用的穩定文字名稱。
 * @param operation Motor CAN 高階操作列舉值。
 * @return 靜態唯讀字串；NONE 或未知值回傳 "IDLE"。
 */
static const char *USB_Command_OperationName(MotorCAN_Operation operation) {
  switch (operation) {
  case MOTOR_CAN_OPERATION_INFO:
    return "INFO";
  case MOTOR_CAN_OPERATION_SET_ID:
    return "SET_ID";
  case MOTOR_CAN_OPERATION_TEST:
    return "TEST";
  case MOTOR_CAN_OPERATION_HOME:
    return "HOME";
  case MOTOR_CAN_OPERATION_INIT:
    return "INIT";
  case MOTOR_CAN_OPERATION_ROTATE:
    return "ROTATE";
  case MOTOR_CAN_OPERATION_PROVISION_1M:
    return "PROVISION_1M";
  default:
    return "IDLE";
  }
}

/**
 * @brief 把 MotorCAN_Error 轉成 USB 文字協定的錯誤名稱。
 * @param error Motor CAN 錯誤列舉值。
 * @return 靜態唯讀字串；未知值回傳 "UNKNOWN"。
 */
static const char *USB_Command_ErrorName(MotorCAN_Error error) {
  switch (error) {
  case MOTOR_CAN_ERROR_TARGET_NOT_FOUND:
    return "TARGET_NOT_FOUND";
  case MOTOR_CAN_ERROR_NEW_ID_IN_USE:
    return "NEW_ID_IN_USE";
  case MOTOR_CAN_ERROR_DEVICE_REJECTED:
    return "DEVICE_REJECTED";
  case MOTOR_CAN_ERROR_SAVE_FAILED:
    return "SAVE_FAILED";
  case MOTOR_CAN_ERROR_VERIFY_FAILED:
    return "VERIFY_FAILED";
  case MOTOR_CAN_ERROR_RESPONSE_TIMEOUT:
    return "RESPONSE_TIMEOUT";
  case MOTOR_CAN_ERROR_RX_OVERFLOW:
    return "CAN_RX_OVERFLOW";
  case MOTOR_CAN_ERROR_BUS:
    return "CAN_BUS_ERROR";
  case MOTOR_CAN_ERROR_TX:
    return "CAN_TX_FAILED";
  case MOTOR_CAN_ERROR_RECONFIG_FAILED:
    return "CAN_RECONFIG_FAILED";
  case MOTOR_CAN_ERROR_HOME_TIMEOUT:
    return "HOME_TIMEOUT";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief 把 MotorCAN_Start*() 的立即狀態轉成 USB 文字協定名稱。
 * @param status Motor CAN 啟動狀態。
 * @return 靜態唯讀字串；未列出的值回傳 "UNKNOWN"。
 * @note MOTOR_CAN_STATUS_OK 由 ReportStartStatus() 直接處理，不需要文字
 * mapping。
 */
static const char *USB_Command_StatusName(MotorCAN_Status status) {
  switch (status) {
  case MOTOR_CAN_STATUS_NOT_READY:
    return "CAN_NOT_READY";
  case MOTOR_CAN_STATUS_BUSY:
    return "BUSY";
  case MOTOR_CAN_STATUS_INVALID_ARGUMENT:
    return "INVALID_ARGUMENT";
  case MOTOR_CAN_STATUS_EMS_ACTIVE:
    return "EMS_ACTIVE";
  case MOTOR_CAN_STATUS_INIT_REQUIRED:
    return "INIT_REQUIRED";
  case MOTOR_CAN_STATUS_TX_FAILED:
    return "CAN_TX_FAILED";
  case MOTOR_CAN_STATUS_RECONFIG_FAILED:
    return "CAN_RECONFIG_FAILED";
  default:
    return "UNKNOWN";
  }
}

/**
 * @brief 解析 uint16_t 範圍內的十進位或 0x 前綴十六進位數字。
 * @param text NUL-terminated token。
 * @param maximum 呼叫端允許的最大值。
 * @param value 成功時接收解析結果。
 * @retval 1 表示整個 token 合法且未超過 maximum；否則為 0。
 */
static uint8_t USB_Command_ParseNumber(const char *text, uint16_t maximum,
                                       uint16_t *value) {
  char *end;
  unsigned long parsed;
  int base = 10;

  if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
    return 0U;
  }
  /* 只有明確 0x/0X 前綴才視為十六進位，避免前導 0 被當成八進位。 */
  if ((text[0] == '0') && ((text[1] == 'X') || (text[1] == 'x'))) {
    base = 16;
  }

  parsed = strtoul(text, &end, base);
  if ((*end != '\0') || (parsed > maximum)) {
    return 0U;
  }

  *value = (uint16_t)parsed;
  return 1U;
}

/**
 * @brief 解析 uint32_t 範圍內的十進位或 0x 前綴十六進位數字。
 * @param text NUL-terminated token；不接受負號。
 * @param maximum 呼叫端允許的最大值。
 * @param value 成功時接收解析結果。
 * @retval 1 表示格式、ERANGE 與 maximum 檢查均通過；否則為 0。
 */
static uint8_t USB_Command_ParseUint32(const char *text, uint32_t maximum,
                                       uint32_t *value) {
  char *end;
  unsigned long parsed;
  int base = 10;

  if ((text == NULL) || (value == NULL) || (text[0] == '\0') ||
      (text[0] == '-')) {
    return 0U;
  }
  if ((text[0] == '0') && ((text[1] == 'X') || (text[1] == 'x'))) {
    base = 16;
  }

  /* strtoul 的 ERANGE 與上限都要檢查，避免平台 long 寬度差異造成截斷。 */
  errno = 0;
  parsed = strtoul(text, &end, base);
  if ((errno == ERANGE) || (*end != '\0') || (parsed > maximum)) {
    return 0U;
  }

  *value = (uint32_t)parsed;
  return 1U;
}

/**
 * @brief 解析十進位 signed 32-bit 整數，供 HOME origin offset 使用。
 * @param text NUL-terminated 十進位 token。
 * @param value 成功時接收 double 結果。
 * @retval 1 表示完整 token 是十進位浮點數；否則為 0。
 */
static uint8_t USB_Command_ParseDouble(const char *text, double *value) {
  char *end;
  double parsed;

  if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
    return 0U;
  }

  errno = 0;
  parsed = strtod(text, &end);
  if ((errno == ERANGE) || (end == text) || (*end != '\0')) {
    return 0U;
  }

  *value = parsed;
  return 1U;
}

/**
 * @brief 將 MotorCAN_Start*() 的立即結果排成 STARTED 或 ERR 回覆。
 * @param status Motor CAN 啟動狀態。
 * @param operation 要放進回覆的命令名稱。
 * @retval 無。
 * @note STARTED 只代表接受要求；真正完成結果會由 MotorCAN event 稍後回報。
 */
static void USB_Command_ReportStartStatus(MotorCAN_Status status,
                                          const char *operation) {
  /* STARTED 只代表 state machine 已接受工作；最終結果會以另一行事件回覆。 */
  if (status == MOTOR_CAN_STATUS_OK) {
    USB_Command_QueueText("OK %s STARTED", operation);
  } else {
    USB_Command_QueueText("ERR %s %s", operation,
                          USB_Command_StatusName(status));
  }
}

/**
 * @brief 檢查 strtok() 目前解析位置後方是否還有多餘 token。
 * @retval 1 表示有額外參數；0 表示參數數量剛好。
 */
static uint8_t USB_Command_HasExtraToken(void) {
  /* 每個命令都要求參數數量精確，避免拼字或多餘參數被靜默忽略。 */
  return (strtok(NULL, " \t") != NULL) ? 1U : 0U;
}

/**
 * @brief 解析並執行一條完整 USB CDC 文字命令。
 * @param line 可修改的 NUL-terminated command line buffer。
 * @retval 無。
 * @note 會原地轉大寫並用 strtok() 切 token；所有輸入先完整驗證才呼叫 MotorCAN。
 */
static void USB_Command_ExecuteLine(char *line) {
  char *command;
  char *token_bus;
  char *token_id;
  char *token_new_id;
  char *token_rate;
  char *token_direction;
  char *token_high_speed;
  char *token_low_speed;
  char *token_offset;
  char *token_timeout;
  char *token_confirm;
  char *token_command;
  char *token_big_offset;
  char *token_small_offset;
  uint16_t bus_value;
  uint16_t id_value;
  uint16_t new_id_value;
  uint16_t rate_value;
  uint16_t high_speed_value;
  uint16_t low_speed_value;
  uint8_t direction_value;
  double offset_value;
  double big_offset_value;
  double small_offset_value;
  uint32_t timeout_value;
  MotorCAN_Status status;
  size_t index;

  /*
   * parser 會原地修改 line：先轉大寫，再由 strtok() 插入 NUL 分割 token。
   * line 位於模組私有 buffer，因此不需要保留原始輸入。
   */
  for (index = 0U; line[index] != '\0'; index++) {
    line[index] = (char)toupper((unsigned char)line[index]);
  }

  command = strtok(line, " \t");
  if (command == NULL) {
    return;
  }

  /* PING/HELP/STATUS 是本機命令，不會送 CAN frame。 */
  if (strcmp(command, "PING") == 0) {
    if (USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR PING SYNTAX");
      return;
    }
    USB_Command_QueueText("OK PONG");
    return;
  }

  if (strcmp(command, "HELP") == 0) {
    USB_Command_QueueText("OK COMMANDS");
    USB_Command_QueueText("INFO <bus:1|2> <id:1..0x7FF>");
    USB_Command_QueueText("CAN_RATE <bus> <500|1000>");
    USB_Command_QueueText("PROVISION_1M <bus> <id> CONFIRM");
    USB_Command_QueueText("SET_ID <bus> <old_id> <new_id> CONFIRM");
    USB_Command_QueueText(
        "INIT [<big_offset_angle_deg> <small_offset_angle_deg>] CONFIRM");
    USB_Command_QueueText("TEST <bus> <id> CONFIRM  (30RPM,500ms)");
    USB_Command_QueueText("HOME <bus> <id> <FWD|REV> <high_rpm> <low_rpm> "
                          "<offset_angle_deg> <timeout_ms> CONFIRM");
    USB_Command_QueueText("STATUS | PING | HELP");
    USB_Command_QueueText("ROTATE <bus> <id> "
                          "<R|R_|L|L_|U|U_|D|D_|F|F_|B|B_|Rw|Rw_|Lw|Lw_|Uw|Uw_|"
                          "Dw|Dw_|Fw|Fw_|Bw|Bw_> CONFIRM");
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    if (USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR STATUS SYNTAX");
      return;
    }
    USB_Command_QueueText("OK STATUS ems=%s initialized=%u operation=%s "
                          "can1=%uK can2=%uK ems_resume_without_reset=1",
                          EMS_IsStopActive() ? "ACTIVE" : "OK",
                          MotorCAN_IsInitialized(),
                          USB_Command_OperationName(MotorCAN_GetOperation()),
                          MotorCAN_GetBusBitrate(1U),
                          MotorCAN_GetBusBitrate(2U));
    return;
  }

  /*
   * EMS 啟動或釋放清理期間，只允許不會操作馬達的本機查詢。
   * RX 仍持續解析，所以上位機不會把「連線正常」誤判成無回應。
   */
  if (EMS_AreCommandsBlocked()) {
    USB_Command_QueueText("ERR %s EMS_ACTIVE", command);
    return;
  }

  /* CAN_RATE 只改 STM32 controller 的 bitrate，不會修改馬達內部設定。 */
  if (strcmp(command, "INIT") == 0) {
    token_big_offset = strtok(NULL, " \t");
    if (token_big_offset == NULL) {
      USB_Command_QueueText("ERR INIT SYNTAX_OR_CONFIRM");
      return;
    }
    if (strcmp(token_big_offset, "CONFIRM") == 0) {
      if (USB_Command_HasExtraToken()) {
        USB_Command_QueueText("ERR INIT SYNTAX_OR_CONFIRM");
        return;
      }
      status = MotorCAN_StartInit();
    } else {
      token_small_offset = strtok(NULL, " \t");
      token_confirm = strtok(NULL, " \t");
      if ((!USB_Command_ParseDouble(token_big_offset, &big_offset_value)) ||
          (!USB_Command_ParseDouble(token_small_offset, &small_offset_value)) ||
          (token_confirm == NULL) ||
          (strcmp(token_confirm, "CONFIRM") != 0) ||
          USB_Command_HasExtraToken()) {
        USB_Command_QueueText("ERR INIT SYNTAX_OR_CONFIRM");
        return;
      }
      status = MotorCAN_StartInitWithOffsetAngles(big_offset_value,
                                                   small_offset_value);
    }
    USB_Command_ReportStartStatus(status, "INIT");
    return;
  }

  if (strcmp(command, "CAN_RATE") == 0) {
    token_bus = strtok(NULL, " \t");
    token_rate = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_rate, 1000U, &rate_value)) ||
        ((rate_value != 500U) && (rate_value != 1000U)) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR CAN_RATE SYNTAX");
      return;
    }
    status = MotorCAN_SetBusBitrate((uint8_t)bus_value, rate_value);
    if (status == MOTOR_CAN_STATUS_OK) {
      USB_Command_QueueText("OK CAN_RATE bus=%u rate=%uK", bus_value,
                            rate_value);
    } else {
      USB_Command_QueueText("ERR CAN_RATE %s", USB_Command_StatusName(status));
    }
    return;
  }

  /* 會改寫馬達設定的命令要求最後一個 token 必須是 CONFIRM。 */
  if (strcmp(command, "PROVISION_1M") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) || (token_confirm == NULL) ||
        (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR PROVISION_1M SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartProvision1M((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "PROVISION_1M");
    return;
  }

  /* INFO 是唯一直連馬達但不改設定、不造成動作的查詢。 */
  if (strcmp(command, "INFO") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) || USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR INFO SYNTAX");
      return;
    }
    status = MotorCAN_StartInfo((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "INFO");
    return;
  }

  if (strcmp(command, "SET_ID") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_new_id = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) ||
        (!USB_Command_ParseNumber(token_new_id, 0x7FFU, &new_id_value)) ||
        (new_id_value < 1U) || (id_value == new_id_value) ||
        (token_confirm == NULL) || (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR SET_ID SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartSetId((uint8_t)bus_value, id_value, new_id_value);
    USB_Command_ReportStartStatus(status, "SET_ID");
    return;
  }

  /* TEST 會造成實際移動，因此也要求 CONFIRM，並由 MotorCAN 再檢查 EMS。 */
  if (strcmp(command, "TEST") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) || (token_confirm == NULL) ||
        (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR TEST SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartTest((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "TEST");
    return;
  }

  /*
   * HOME 同時改寫 homing 參數並驅動馬達。方向先獨立解析，再一次驗證
   * 所有數值範圍、速度關係、timeout 與 CONFIRM，成功後才啟動 MotorCAN。
   */
  if (strcmp(command, "HOME") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_direction = strtok(NULL, " \t");
    token_high_speed = strtok(NULL, " \t");
    token_low_speed = strtok(NULL, " \t");
    token_offset = strtok(NULL, " \t");
    token_timeout = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");

    if ((token_direction != NULL) && (strcmp(token_direction, "FWD") == 0)) {
      direction_value = 0U;
    } else if ((token_direction != NULL) &&
               (strcmp(token_direction, "REV") == 0)) {
      direction_value = 1U;
    } else {
      USB_Command_QueueText("ERR HOME SYNTAX_OR_CONFIRM");
      return;
    }

    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) ||
        (!USB_Command_ParseNumber(token_high_speed, 3000U,
                                  &high_speed_value)) ||
        (high_speed_value < 1U) ||
        (!USB_Command_ParseNumber(token_low_speed, 100U, &low_speed_value)) ||
        (low_speed_value < 1U) || (low_speed_value > high_speed_value) ||
        (!USB_Command_ParseDouble(token_offset, &offset_value)) ||
        (!USB_Command_ParseUint32(token_timeout, 120000U, &timeout_value)) ||
        (timeout_value < 1000U) || (token_confirm == NULL) ||
        (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR HOME SYNTAX_OR_CONFIRM");
      return;
    }

    status = MotorCAN_StartHome((uint8_t)bus_value, id_value, direction_value,
                                high_speed_value, low_speed_value, offset_value,
                                timeout_value);
    USB_Command_ReportStartStatus(status, "HOME");
    return;
  }

  if (strcmp(command, "ROTATE") == 0) {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_command = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");

    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) || (token_command == NULL) || (token_confirm == NULL) ||
        (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken()) {
      USB_Command_QueueText("ERR ROTATE SYNTAX_OR_CONFIRM");
      return;
    }

    status = MotorCAN_StartRotate((uint8_t)bus_value, id_value, token_command);
    USB_Command_ReportStartStatus(status, "ROTATE");
    return;
  }

  USB_Command_QueueText("ERR UNKNOWN_COMMAND; SEND HELP");
}

/**
 * @brief 從 RX byte ring buffer 組出命令列，遇到 CR/LF 時交給 ExecuteLine()。
 * @retval 無。
 * @note 忽略非 printable ASCII；行過長與 RX overflow 會各自產生錯誤回覆。
 */
static void USB_Command_ProcessRx(void) {
  /*
   * CDC 是 byte stream，一個 USB packet 可能包含半行、多行或行尾拆包。
   * 因此逐 byte 累積，遇到 CR 或 LF 才執行完整命令。連續 CRLF 的第二個
   * 分隔字元會因 line_length=0 而被忽略。
   */
  while (usb_rx_tail != usb_rx_head) {
    uint8_t byte = usb_rx_queue[usb_rx_tail];
    usb_rx_tail = (uint16_t)((usb_rx_tail + 1U) % USB_COMMAND_RX_QUEUE_SIZE);

    if ((byte == '\r') || (byte == '\n')) {
      if (usb_line_overflow) {
        USB_Command_QueueText("ERR LINE_TOO_LONG");
      } else if (usb_line_length > 0U) {
        usb_line[usb_line_length] = '\0';
        USB_Command_ExecuteLine(usb_line);
      }
      usb_line_length = 0U;
      usb_line_overflow = 0U;
      continue;
    }

    /* 命令協定只接受 printable ASCII；控制字元除了 CR/LF 全部忽略。 */
    if ((byte < 0x20U) || (byte > 0x7EU)) {
      continue;
    }

    if (usb_line_length < (USB_COMMAND_LINE_SIZE - 1U)) {
      usb_line[usb_line_length++] = (char)byte;
    } else {
      usb_line_overflow = 1U;
    }
  }

  if (usb_rx_overflow) {
    usb_rx_overflow = 0U;
    USB_Command_QueueText("ERR USB_RX_OVERFLOW");
  }
}

/**
 * @brief 取出所有 MotorCAN_Event，轉成 USB 文字協定並排入 TX queue。
 * @retval 無。
 * @note 會處理 INFO、SET_ID、PROVISION_1M、TEST、HOME、ROTATE 與一般 CAN 錯誤。
 */
static void USB_Command_ProcessMotorEvents(void) {
  /* 將 MotorCAN 的結構化事件轉成穩定、方便上位機解析的一行式文字協定。 */
  MotorCAN_Event event;

  while (MotorCAN_GetEvent(&event)) {
    switch (event.type) {
    case MOTOR_CAN_EVENT_INFO:
      USB_Command_QueueText(
          "OK INFO bus=%u id=0x%03X hardware=%u firmware=%u.%u.%u", event.bus,
          event.id, event.hardware_version, event.firmware_version[0],
          event.firmware_version[1], event.firmware_version[2]);
      break;

    case MOTOR_CAN_EVENT_ID_CHANGED:
      USB_Command_QueueText(
          "OK SET_ID bus=%u old=0x%03X new=0x%03X saved=1 verified=1",
          event.bus, event.old_id, event.new_id);
      break;

    case MOTOR_CAN_EVENT_MOTOR_RATE_1M:
      USB_Command_QueueText("OK PROVISION_1M bus=%u id=0x%03X motor_rate=1000K "
                            "stm32_rate=1000K saved=1 restarted=1 verified=1",
                            event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_TEST_FINISHED:
      USB_Command_QueueText(
          "OK TEST bus=%u id=0x%03X speed=30RPM duration=500ms complete",
          event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS:
      USB_Command_QueueText(
          "ERR TEST EMS_ACTIVE bus=%u id=0x%03X control_stop_sent=1", event.bus,
          event.id);
      break;

    case MOTOR_CAN_EVENT_HOME_FINISHED:
      USB_Command_QueueText("OK HOME bus=%u id=0x%03X complete enabled=1",
                            event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_HOME_STOPPED_BY_EMS:
      USB_Command_QueueText(
          "ERR HOME EMS_ACTIVE bus=%u id=0x%03X control_stop_sent=1 disabled=1",
          event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_INIT_BIG_PROGRESS:
      USB_Command_QueueText(
          "PROGRESS INIT big bus=%u id=0x%03X completed=%u/6 mask=0x%02X",
          event.bus, event.id, event.completed_count, event.completed_mask);
      break;

    case MOTOR_CAN_EVENT_INIT_FINISHED:
      USB_Command_QueueText(
          "OK INIT big=6 synchronized=1 small=6 sequential=1 complete");
      break;

    case MOTOR_CAN_EVENT_INIT_STOPPED_BY_EMS:
      USB_Command_QueueText(
          "ERR INIT EMS_ACTIVE control_stop_sent=1 disabled=1");
      break;

    case MOTOR_CAN_EVENT_ROTATE_FINISHED:
      USB_Command_QueueText("OK ROTATE bus=%u id=0x%03X synchronized=1 complete",
                            event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_ROTATE_STOPPED_BY_EMS:
      USB_Command_QueueText(
          "ERR ROTATE EMS_ACTIVE bus=%u id=0x%03X control_stop_sent=1 disabled=1",
          event.bus, event.id);
      break;

    case MOTOR_CAN_EVENT_ERROR:
      if ((event.operation == MOTOR_CAN_OPERATION_INIT) &&
          (event.missing_mask != 0U)) {
        USB_Command_QueueText(
            "ERR CAN %s bus=%u id=0x%03X completed_mask=0x%02X missing_mask=0x%02X",
            USB_Command_ErrorName(event.error), event.bus, event.id,
            event.completed_mask, event.missing_mask);
      } else {
        USB_Command_QueueText(
            "ERR CAN %s bus=%u id=0x%03X old=0x%03X new=0x%03X",
            USB_Command_ErrorName(event.error), event.bus, event.id,
            event.old_id, event.new_id);
      }
      break;

    default:
      break;
    }
  }
}

/**
 * @brief 若 USB 沒有傳送中的封包，嘗試送出 TX queue 最舊的一行。
 * @retval 無。
 * @note USBD_BUSY/FAIL 時保留原 tail，下一輪主迴圈會重試同一筆資料。
 */
static void USB_Command_ProcessTx(void) {
  uint8_t result;

  if (usb_tx_inflight || (usb_tx_tail == usb_tx_head)) {
    return;
  }

  /*
   * 先標示 inflight，再啟動 USB 傳送，避免極短封包的 complete callback
   * 早於狀態更新。USBD_BUSY 時保留同一個 tail，下一輪主迴圈再重試。
   */
  usb_tx_inflight = 1U;
  result = CDC_Transmit_FS((uint8_t *)usb_tx_queue[usb_tx_tail],
                           (uint16_t)strlen(usb_tx_queue[usb_tx_tail]));
  if (result != USBD_OK) {
    usb_tx_inflight = 0U;
  }
}

/**
 * @brief 將 USB command 模組的 RX、line parser 與 TX queue 重設為空。
 * @retval 無。
 * @note 在啟動 USB Device 前呼叫，確保列舉 callback 看到已初始化狀態。
 */
void USB_Command_Init(void) {
  usb_rx_head = 0U;
  usb_rx_tail = 0U;
  usb_rx_overflow = 0U;
  usb_tx_head = 0U;
  usb_tx_tail = 0U;
  usb_tx_inflight = 0U;
  usb_line_length = 0U;
  usb_line_overflow = 0U;
}

/**
 * @brief 在主迴圈非阻塞地處理 RX 命令、Motor CAN 事件與 TX 傳送。
 * @retval 無。
 * @note EMS blocked 時仍解析本機查詢；會操作馬達的命令由 parser 拒絕。
 */
void USB_Command_Process(void) {
  /*
   * RX parser 必須持續運作，讓 PING/HELP/STATUS 在 EMS 期間仍可回覆；
   * 其他命令會在 USB_Command_ExecuteLine() 回覆 EMS_ACTIVE。
   */
  USB_Command_ProcessRx();
  USB_Command_ProcessMotorEvents();
  USB_Command_ProcessTx();
}

/**
 * @brief EMS 釋放後丟棄尚未解析的 RX bytes 與半條 command line。
 * @retval 無。
 * @note 以保留 PRIMASK 的臨界區同步 USB callback 寫入的 RX head。
 */
void USB_Command_ClearPendingCommands(void) {
  uint32_t primask = __get_PRIMASK();

  /* RX head 由 USB callback 更新；以 PRIMASK 保護 head/tail 同步。 */
  __disable_irq();
  usb_rx_tail = usb_rx_head;
  usb_rx_overflow = 0U;
  if (primask == 0U) {
    __enable_irq();
  }

  usb_line_length = 0U;
  usb_line_overflow = 0U;
}

/**
 * @brief 接收 CDC OUT packet，逐 byte 寫入 USB command RX ring buffer。
 * @param data CDC class 提供的 packet buffer。
 * @param length packet 內有效 byte 數。
 * @retval 無。
 * @note 在 USB callback context 執行；只搬移資料，queue 滿時設 overflow。
 */
void USB_Command_OnReceive(const uint8_t *data, uint32_t length) {
  uint32_t index;

  if (data == NULL) {
    return;
  }
  for (index = 0U; index < length; index++) {
    uint16_t next = (uint16_t)((usb_rx_head + 1U) % USB_COMMAND_RX_QUEUE_SIZE);
    if (next == usb_rx_tail) {
      usb_rx_overflow = 1U;
      break;
    }
    usb_rx_queue[usb_rx_head] = data[index];
    usb_rx_head = next;
  }
}

/**
 * @brief 通知 command 層目前 CDC IN transfer 已完成，可釋放 TX tail slot。
 * @retval 無。
 * @note 在 USB transmit-complete callback context 執行。
 */
void USB_Command_OnTransmitComplete(void) {
  /* USB peripheral 已不再引用 tail slot，現在才能釋放並讓下一筆傳送。 */
  if (usb_tx_inflight) {
    usb_tx_tail = (uint8_t)((usb_tx_tail + 1U) % USB_COMMAND_TX_QUEUE_SIZE);
    usb_tx_inflight = 0U;
  }
}

/**
 * @brief USB 中斷連線或 CDC class deinit 時清空所有待傳回覆與 inflight 狀態。
 * @retval 無。
 * @note RX queue 由 CDC 重新連線後的資料自然重建，此函式只處理 TX lifecycle。
 */
void USB_Command_OnDisconnect(void) {
  /* 斷線後舊回覆沒有收件者，全部捨棄；重新連線從乾淨 TX 狀態開始。 */
  usb_tx_head = 0U;
  usb_tx_tail = 0U;
  usb_tx_inflight = 0U;
}
