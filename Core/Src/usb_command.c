#include "usb_command.h"

#include "main.h"
#include "motor_can.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USB_COMMAND_RX_QUEUE_SIZE     256U
#define USB_COMMAND_LINE_SIZE         96U
#define USB_COMMAND_TX_QUEUE_SIZE     8U
#define USB_COMMAND_TX_MESSAGE_SIZE   192U

/* RX queue 由 USB ISR 寫入、主迴圈讀出。 */
static uint8_t usb_rx_queue[USB_COMMAND_RX_QUEUE_SIZE];
static volatile uint16_t usb_rx_head;
static volatile uint16_t usb_rx_tail;
static volatile uint8_t usb_rx_overflow;

/* 每個 TX slot 在 transmit complete 前都不會重複使用。 */
static char usb_tx_queue[USB_COMMAND_TX_QUEUE_SIZE][USB_COMMAND_TX_MESSAGE_SIZE];
static volatile uint8_t usb_tx_head;
static volatile uint8_t usb_tx_tail;
static volatile uint8_t usb_tx_inflight;

static char usb_line[USB_COMMAND_LINE_SIZE];
static uint16_t usb_line_length;
static uint8_t usb_line_overflow;

static void USB_Command_QueueText(const char *format, ...)
{
  uint8_t next;
  int length;
  va_list arguments;

  next = (uint8_t)((usb_tx_head + 1U) % USB_COMMAND_TX_QUEUE_SIZE);
  if (next == usb_tx_tail)
  {
    /* USB host 未讀取時不阻塞主迴圈，保留既有回覆。 */
    return;
  }

  va_start(arguments, format);
  length = vsnprintf(usb_tx_queue[usb_tx_head],
                     USB_COMMAND_TX_MESSAGE_SIZE - 3U,
                     format, arguments);
  va_end(arguments);

  if (length < 0)
  {
    return;
  }
  if (length > (int)(USB_COMMAND_TX_MESSAGE_SIZE - 4U))
  {
    /* vsnprintf 已在此位置補 NUL；改寫成 CRLF 後仍保留結尾 NUL。 */
    length = (int)(USB_COMMAND_TX_MESSAGE_SIZE - 4U);
  }

  usb_tx_queue[usb_tx_head][length++] = '\r';
  usb_tx_queue[usb_tx_head][length++] = '\n';
  usb_tx_queue[usb_tx_head][length] = '\0';
  usb_tx_head = next;
}

static const char *USB_Command_OperationName(MotorCAN_Operation operation)
{
  switch (operation)
  {
    case MOTOR_CAN_OPERATION_INFO:
      return "INFO";
    case MOTOR_CAN_OPERATION_SET_ID:
      return "SET_ID";
    case MOTOR_CAN_OPERATION_TEST:
      return "TEST";
    case MOTOR_CAN_OPERATION_PROVISION_1M:
      return "PROVISION_1M";
    default:
      return "IDLE";
  }
}

static const char *USB_Command_ErrorName(MotorCAN_Error error)
{
  switch (error)
  {
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
    default:
      return "UNKNOWN";
  }
}

static const char *USB_Command_StatusName(MotorCAN_Status status)
{
  switch (status)
  {
    case MOTOR_CAN_STATUS_NOT_READY:
      return "CAN_NOT_READY";
    case MOTOR_CAN_STATUS_BUSY:
      return "BUSY";
    case MOTOR_CAN_STATUS_INVALID_ARGUMENT:
      return "INVALID_ARGUMENT";
    case MOTOR_CAN_STATUS_EMS_LATCHED:
      return "EMS_LATCHED";
    case MOTOR_CAN_STATUS_TX_FAILED:
      return "CAN_TX_FAILED";
    case MOTOR_CAN_STATUS_RECONFIG_FAILED:
      return "CAN_RECONFIG_FAILED";
    default:
      return "UNKNOWN";
  }
}

static uint8_t USB_Command_ParseNumber(const char *text, uint16_t maximum,
                                       uint16_t *value)
{
  char *end;
  unsigned long parsed;
  int base = 10;

  if ((text == NULL) || (value == NULL) || (text[0] == '\0'))
  {
    return 0U;
  }
  if ((text[0] == '0') && ((text[1] == 'X') || (text[1] == 'x')))
  {
    base = 16;
  }

  parsed = strtoul(text, &end, base);
  if ((*end != '\0') || (parsed > maximum))
  {
    return 0U;
  }

  *value = (uint16_t)parsed;
  return 1U;
}

static void USB_Command_ReportStartStatus(MotorCAN_Status status,
                                          const char *operation)
{
  if (status == MOTOR_CAN_STATUS_OK)
  {
    USB_Command_QueueText("OK %s STARTED", operation);
  }
  else
  {
    USB_Command_QueueText("ERR %s %s", operation,
                          USB_Command_StatusName(status));
  }
}

static uint8_t USB_Command_HasExtraToken(void)
{
  return (strtok(NULL, " \t") != NULL) ? 1U : 0U;
}

static void USB_Command_ExecuteLine(char *line)
{
  char *command;
  char *token_bus;
  char *token_id;
  char *token_new_id;
  char *token_rate;
  char *token_confirm;
  uint16_t bus_value;
  uint16_t id_value;
  uint16_t new_id_value;
  uint16_t rate_value;
  MotorCAN_Status status;
  size_t index;

  /* 命令不分大小寫；數字中的 0x 轉成 0X 仍可解析。 */
  for (index = 0U; line[index] != '\0'; index++)
  {
    line[index] = (char)toupper((unsigned char)line[index]);
  }

  command = strtok(line, " \t");
  if (command == NULL)
  {
    return;
  }

  if (strcmp(command, "PING") == 0)
  {
    if (USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR PING SYNTAX");
      return;
    }
    USB_Command_QueueText("OK PONG");
    return;
  }

  if (strcmp(command, "HELP") == 0)
  {
    USB_Command_QueueText("OK COMMANDS");
    USB_Command_QueueText("INFO <bus:1|2> <id:1..0x7FF>");
    USB_Command_QueueText("CAN_RATE <bus> <500|1000>");
    USB_Command_QueueText("PROVISION_1M <bus> <id> CONFIRM");
    USB_Command_QueueText("SET_ID <bus> <old_id> <new_id> CONFIRM");
    USB_Command_QueueText("TEST <bus> <id> CONFIRM  (30RPM,500ms)");
    USB_Command_QueueText("STATUS | PING | HELP");
    return;
  }

  if (strcmp(command, "STATUS") == 0)
  {
    if (USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR STATUS SYNTAX");
      return;
    }
    USB_Command_QueueText(
                           "OK STATUS ems=%s operation=%s can1=%uK can2=%uK ems_control_only=1",
                           EMS_IsStopLatched() ? "LATCHED" : "OK",
                           USB_Command_OperationName(MotorCAN_GetOperation()),
                           MotorCAN_GetBusBitrate(1U),
                           MotorCAN_GetBusBitrate(2U));
    return;
  }

  if (strcmp(command, "CAN_RATE") == 0)
  {
    token_bus = strtok(NULL, " \t");
    token_rate = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_rate, 1000U, &rate_value)) ||
        ((rate_value != 500U) && (rate_value != 1000U)) ||
        USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR CAN_RATE SYNTAX");
      return;
    }
    status = MotorCAN_SetBusBitrate((uint8_t)bus_value, rate_value);
    if (status == MOTOR_CAN_STATUS_OK)
    {
      USB_Command_QueueText("OK CAN_RATE bus=%u rate=%uK", bus_value, rate_value);
    }
    else
    {
      USB_Command_QueueText("ERR CAN_RATE %s", USB_Command_StatusName(status));
    }
    return;
  }

  if (strcmp(command, "PROVISION_1M") == 0)
  {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) ||
        (token_confirm == NULL) || (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR PROVISION_1M SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartProvision1M((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "PROVISION_1M");
    return;
  }

  if (strcmp(command, "INFO") == 0)
  {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) || USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR INFO SYNTAX");
      return;
    }
    status = MotorCAN_StartInfo((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "INFO");
    return;
  }

  if (strcmp(command, "SET_ID") == 0)
  {
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
        USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR SET_ID SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartSetId((uint8_t)bus_value, id_value, new_id_value);
    USB_Command_ReportStartStatus(status, "SET_ID");
    return;
  }

  if (strcmp(command, "TEST") == 0)
  {
    token_bus = strtok(NULL, " \t");
    token_id = strtok(NULL, " \t");
    token_confirm = strtok(NULL, " \t");
    if ((!USB_Command_ParseNumber(token_bus, 2U, &bus_value)) ||
        (bus_value < 1U) ||
        (!USB_Command_ParseNumber(token_id, 0x7FFU, &id_value)) ||
        (id_value < 1U) ||
        (token_confirm == NULL) || (strcmp(token_confirm, "CONFIRM") != 0) ||
        USB_Command_HasExtraToken())
    {
      USB_Command_QueueText("ERR TEST SYNTAX_OR_CONFIRM");
      return;
    }
    status = MotorCAN_StartTest((uint8_t)bus_value, id_value);
    USB_Command_ReportStartStatus(status, "TEST");
    return;
  }

  USB_Command_QueueText("ERR UNKNOWN_COMMAND; SEND HELP");
}

static void USB_Command_ProcessRx(void)
{
  while (usb_rx_tail != usb_rx_head)
  {
    uint8_t byte = usb_rx_queue[usb_rx_tail];
    usb_rx_tail = (uint16_t)((usb_rx_tail + 1U) % USB_COMMAND_RX_QUEUE_SIZE);

    if ((byte == '\r') || (byte == '\n'))
    {
      if (usb_line_overflow)
      {
        USB_Command_QueueText("ERR LINE_TOO_LONG");
      }
      else if (usb_line_length > 0U)
      {
        usb_line[usb_line_length] = '\0';
        USB_Command_ExecuteLine(usb_line);
      }
      usb_line_length = 0U;
      usb_line_overflow = 0U;
      continue;
    }

    if ((byte < 0x20U) || (byte > 0x7EU))
    {
      continue;
    }

    if (usb_line_length < (USB_COMMAND_LINE_SIZE - 1U))
    {
      usb_line[usb_line_length++] = (char)byte;
    }
    else
    {
      usb_line_overflow = 1U;
    }
  }

  if (usb_rx_overflow)
  {
    usb_rx_overflow = 0U;
    USB_Command_QueueText("ERR USB_RX_OVERFLOW");
  }
}

static void USB_Command_ProcessMotorEvents(void)
{
  MotorCAN_Event event;

  while (MotorCAN_GetEvent(&event))
  {
    switch (event.type)
    {
      case MOTOR_CAN_EVENT_INFO:
        USB_Command_QueueText(
          "OK INFO bus=%u id=0x%03X hardware=%u firmware=%u.%u.%u",
          event.bus, event.id, event.hardware_version,
          event.firmware_version[0], event.firmware_version[1],
          event.firmware_version[2]);
        break;

      case MOTOR_CAN_EVENT_ID_CHANGED:
        USB_Command_QueueText(
          "OK SET_ID bus=%u old=0x%03X new=0x%03X saved=1 verified=1",
          event.bus, event.old_id, event.new_id);
        break;

      case MOTOR_CAN_EVENT_MOTOR_RATE_1M:
        USB_Command_QueueText(
          "OK PROVISION_1M bus=%u id=0x%03X motor_rate=1000K stm32_rate=1000K saved=1 restarted=1 verified=1",
          event.bus, event.id);
        break;

      case MOTOR_CAN_EVENT_TEST_FINISHED:
        USB_Command_QueueText(
          "OK TEST bus=%u id=0x%03X speed=30RPM duration=500ms complete",
          event.bus, event.id);
        break;

      case MOTOR_CAN_EVENT_TEST_STOPPED_BY_EMS:
        USB_Command_QueueText(
          "ERR TEST EMS_LATCHED bus=%u id=0x%03X control_stop_sent=1",
          event.bus, event.id);
        break;

      case MOTOR_CAN_EVENT_ERROR:
        USB_Command_QueueText(
          "ERR CAN %s bus=%u id=0x%03X old=0x%03X new=0x%03X",
          USB_Command_ErrorName(event.error), event.bus, event.id,
          event.old_id, event.new_id);
        break;

      default:
        break;
    }
  }
}

static void USB_Command_ProcessTx(void)
{
  uint8_t result;

  if (usb_tx_inflight || (usb_tx_tail == usb_tx_head))
  {
    return;
  }

  /* 先標示 inflight，避免極短封包完成中斷早於狀態更新。 */
  usb_tx_inflight = 1U;
  result = CDC_Transmit_FS((uint8_t *)usb_tx_queue[usb_tx_tail],
                           (uint16_t)strlen(usb_tx_queue[usb_tx_tail]));
  if (result != USBD_OK)
  {
    usb_tx_inflight = 0U;
  }
}

void USB_Command_Init(void)
{
  usb_rx_head = 0U;
  usb_rx_tail = 0U;
  usb_rx_overflow = 0U;
  usb_tx_head = 0U;
  usb_tx_tail = 0U;
  usb_tx_inflight = 0U;
  usb_line_length = 0U;
  usb_line_overflow = 0U;
}

void USB_Command_Process(void)
{
  USB_Command_ProcessRx();
  USB_Command_ProcessMotorEvents();
  USB_Command_ProcessTx();
}

void USB_Command_OnReceive(const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if (data == NULL)
  {
    return;
  }

  for (index = 0U; index < length; index++)
  {
    uint16_t next = (uint16_t)((usb_rx_head + 1U) % USB_COMMAND_RX_QUEUE_SIZE);
    if (next == usb_rx_tail)
    {
      usb_rx_overflow = 1U;
      break;
    }
    usb_rx_queue[usb_rx_head] = data[index];
    usb_rx_head = next;
  }
}

void USB_Command_OnTransmitComplete(void)
{
  if (usb_tx_inflight)
  {
    usb_tx_tail = (uint8_t)((usb_tx_tail + 1U) % USB_COMMAND_TX_QUEUE_SIZE);
    usb_tx_inflight = 0U;
  }
}

void USB_Command_OnDisconnect(void)
{
  usb_tx_head = 0U;
  usb_tx_tail = 0U;
  usb_tx_inflight = 0U;
}
