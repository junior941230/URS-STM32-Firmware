#include "ssd1306_status.h"

#include "main.h"
#include "motor_can.h"

#include <stdio.h>
#include <string.h>

#define SSD1306_WIDTH 128U
#define SSD1306_PAGE_COUNT 8U
#define SSD1306_RENDER_INTERVAL_MS 100U
#define SSD1306_RETRY_INTERVAL_MS 1000U
#define SSD1306_I2C_TIMEOUT_MS 5U
#define SSD1306_BOOT_FRAME_INTERVAL_MS 100U
#define SSD1306_BOOT_DURATION_MS 1700U
#define SSD1306_BOOT_GRID_SIZE 4U
#define SSD1306_BOOT_CELL_COUNT 16U

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t online;
  uint8_t next_page;
  uint8_t boot_active;
  uint8_t boot_completed;
  uint8_t timer_running;
  uint8_t frame[SSD1306_PAGE_COUNT][SSD1306_WIDTH];
  uint8_t sent_frame[SSD1306_PAGE_COUNT][SSD1306_WIDTH];
  uint32_t boot_started;
  uint32_t timer_started;
  uint32_t render_deadline;
  uint32_t retry_deadline;
} SSD1306_StatusContext;

static SSD1306_StatusContext display_context;

/* 5x7 字型只保留畫面會使用的 ASCII：數字、大寫英文字母與簡單符號。 */
static const uint8_t digit_font[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU},
};

static const uint8_t uppercase_font[26][5] = {
    {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
    {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
    {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
    {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
    {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
    {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
    {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
    {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
    {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
    {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
    {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
    {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
    {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
    {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
    {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
    {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
    {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
    {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
    {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
    {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
    {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
    {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
    {0x61U, 0x51U, 0x49U, 0x45U, 0x43U},
};

static uint8_t SSD1306_DeadlineReached(uint32_t now, uint32_t deadline) {
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static const uint8_t *SSD1306_GetGlyph(char character) {
  static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
  static const uint8_t period[5] = {0U, 0x60U, 0x60U, 0U, 0U};

  if ((character >= '0') && (character <= '9')) {
    return digit_font[(uint8_t)(character - '0')];
  }
  if ((character >= 'A') && (character <= 'Z')) {
    return uppercase_font[(uint8_t)(character - 'A')];
  }
  if (character == ':') {
    return colon;
  }
  if (character == '.') {
    return period;
  }
  return blank;
}

static void SSD1306_SetPixel(uint8_t x, uint8_t y) {
  if ((x >= SSD1306_WIDTH) || (y >= (SSD1306_PAGE_COUNT * 8U))) {
    return;
  }
  display_context.frame[y / 8U][x] |= (uint8_t)(1U << (y % 8U));
}

static void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t width,
                             uint8_t height) {
  uint16_t draw_x;
  uint16_t draw_y;

  for (draw_y = y; draw_y < ((uint16_t)y + height); draw_y++) {
    for (draw_x = x; draw_x < ((uint16_t)x + width); draw_x++) {
      SSD1306_SetPixel((uint8_t)draw_x, (uint8_t)draw_y);
    }
  }
}

static void SSD1306_DrawRect(uint8_t x, uint8_t y, uint8_t width,
                             uint8_t height) {
  uint16_t offset;

  if ((width == 0U) || (height == 0U)) {
    return;
  }
  for (offset = 0U; offset < width; offset++) {
    SSD1306_SetPixel((uint8_t)(x + offset), y);
    SSD1306_SetPixel((uint8_t)(x + offset),
                     (uint8_t)(y + height - 1U));
  }
  for (offset = 0U; offset < height; offset++) {
    SSD1306_SetPixel(x, (uint8_t)(y + offset));
    SSD1306_SetPixel((uint8_t)(x + width - 1U),
                     (uint8_t)(y + offset));
  }
}

static void SSD1306_DrawTextAt(uint8_t page, uint8_t x, const char *text) {
  uint16_t cursor = x;

  if ((page >= SSD1306_PAGE_COUNT) || (text == NULL)) {
    return;
  }
  while ((*text != '\0') && (cursor <= (SSD1306_WIDTH - 5U))) {
    const uint8_t *glyph = SSD1306_GetGlyph(*text);
    uint8_t column;

    for (column = 0U; column < 5U; column++) {
      display_context.frame[page][cursor + column] = glyph[column];
    }
    cursor += 6U;
    text++;
  }
}

static void SSD1306_DrawTextRightAligned(uint8_t page, const char *text) {
  size_t length;
  size_t width;
  uint8_t x;

  if (text == NULL) {
    return;
  }
  length = strlen(text);
  width = (length > 0U) ? (length * 6U - 1U) : 0U;
  x = (width < SSD1306_WIDTH) ? (uint8_t)(SSD1306_WIDTH - width) : 0U;

  SSD1306_DrawTextAt(page, x, text);
}

static void SSD1306_DrawLargeTextCentered(uint8_t page, const char *text) {
  const size_t length = strlen(text);
  const size_t width = (length > 0U) ? (length * 12U - 2U) : 0U;
  uint16_t cursor =
      (width < SSD1306_WIDTH) ? (uint16_t)((SSD1306_WIDTH - width) / 2U) : 0U;

  if ((page >= (SSD1306_PAGE_COUNT - 1U)) || (text == NULL)) {
    return;
  }
  while ((*text != '\0') && ((cursor + 9U) < SSD1306_WIDTH)) {
    const uint8_t *glyph = SSD1306_GetGlyph(*text);
    uint8_t column;

    for (column = 0U; column < 5U; column++) {
      uint8_t bit;
      for (bit = 0U; bit < 7U; bit++) {
        if ((glyph[column] & (uint8_t)(1U << bit)) != 0U) {
          const uint8_t x = (uint8_t)(cursor + column * 2U);
          const uint8_t y = (uint8_t)(page * 8U + bit * 2U);
          SSD1306_SetPixel(x, y);
          SSD1306_SetPixel((uint8_t)(x + 1U), y);
          SSD1306_SetPixel(x, (uint8_t)(y + 1U));
          SSD1306_SetPixel((uint8_t)(x + 1U), (uint8_t)(y + 1U));
        }
      }
    }
    cursor += 12U;
    text++;
  }
}

static const char *SSD1306_OperationName(MotorCAN_Operation operation) {
  switch (operation) {
  case MOTOR_CAN_OPERATION_INFO:
    return "INFO";
  case MOTOR_CAN_OPERATION_SET_ID:
    return "SET ID";
  case MOTOR_CAN_OPERATION_TEST:
    return "TEST";
  case MOTOR_CAN_OPERATION_HOME:
    return "HOME";
  case MOTOR_CAN_OPERATION_INIT:
    return "INIT";
  case MOTOR_CAN_OPERATION_ROTATE:
    return "ROTATE";
  case MOTOR_CAN_OPERATION_PROVISION_1M:
    return "RATE 1M";
  case MOTOR_CAN_OPERATION_NONE:
  default:
    return "IDLE";
  }
}

static void SSD1306_RenderStatus(void) {
  char elapsed_text[12];

  memset(display_context.frame, 0, sizeof(display_context.frame));

  SSD1306_DrawTextRightAligned(0U,
                               EMS_IsStopActive() ? "EMS STOP" : "EMS OK");
  if (display_context.timer_running) {
    uint32_t elapsed_ms = HAL_GetTick() - display_context.timer_started;
    uint32_t minutes = elapsed_ms / 60000U;
    uint32_t seconds = (elapsed_ms / 1000U) % 60U;
    uint32_t milliseconds = elapsed_ms % 1000U;

    if (minutes > 999U) {
      minutes = 999U;
      seconds = 59U;
      milliseconds = 999U;
    }
    SSD1306_DrawTextAt(
        0U, 0U, SSD1306_OperationName(MotorCAN_GetOperation()));
    (void)snprintf(elapsed_text, sizeof(elapsed_text), "%02lu:%02lu.%03lu",
                   (unsigned long)minutes, (unsigned long)seconds,
                   (unsigned long)milliseconds);
    SSD1306_DrawLargeTextCentered(3U, elapsed_text);
  } else {
    SSD1306_DrawLargeTextCentered(
        3U, SSD1306_OperationName(MotorCAN_GetOperation()));
  }
}

static void SSD1306_RenderBootAnimation(uint8_t animation_frame) {
  const uint8_t filled_cells =
      (animation_frame < SSD1306_BOOT_CELL_COUNT)
          ? animation_frame
          : SSD1306_BOOT_CELL_COUNT;
  const uint8_t bar_width =
      (uint8_t)(((uint16_t)filled_cells * 92U) /
                SSD1306_BOOT_CELL_COUNT);
  uint8_t cell;

  memset(display_context.frame, 0, sizeof(display_context.frame));
  for (cell = 0U; cell < SSD1306_BOOT_CELL_COUNT; cell++) {
    const uint8_t x =
        (uint8_t)(50U + (cell % SSD1306_BOOT_GRID_SIZE) * 7U);
    const uint8_t y =
        (uint8_t)(1U + (cell / SSD1306_BOOT_GRID_SIZE) * 7U);
    SSD1306_DrawRect(x, y, 6U, 6U);
    if (cell < filled_cells) {
      SSD1306_FillRect((uint8_t)(x + 2U), (uint8_t)(y + 2U), 2U, 2U);
    }
  }

  SSD1306_DrawLargeTextCentered(4U, "URS");
  SSD1306_DrawRect(16U, 54U, 96U, 8U);
  if (bar_width > 0U) {
    SSD1306_FillRect(18U, 56U, bar_width, 4U);
  }
}

static HAL_StatusTypeDef SSD1306_WriteCommands(const uint8_t *commands,
                                                uint16_t length) {
  return HAL_I2C_Mem_Write(display_context.i2c,
                           (uint16_t)(SSD1306_I2C_ADDRESS << 1U), 0x00U,
                           I2C_MEMADD_SIZE_8BIT, (uint8_t *)commands, length,
                           SSD1306_I2C_TIMEOUT_MS);
}

static uint8_t SSD1306_TryInitialize(uint32_t now) {
  static const uint8_t init_commands[] = {
      0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U, 0x8DU,
      0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0x7FU,
      0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U, 0xAFU,
  };

  if ((display_context.i2c == NULL) ||
      (HAL_I2C_IsDeviceReady(display_context.i2c,
                             (uint16_t)(SSD1306_I2C_ADDRESS << 1U), 1U,
                             SSD1306_I2C_TIMEOUT_MS) != HAL_OK) ||
      (SSD1306_WriteCommands(init_commands, sizeof(init_commands)) != HAL_OK)) {
    display_context.online = 0U;
    display_context.retry_deadline = now + SSD1306_RETRY_INTERVAL_MS;
    return 0U;
  }

  display_context.online = 1U;
  display_context.next_page = 0U;
  display_context.render_deadline = now;
  if (!display_context.boot_completed) {
    display_context.boot_active = 1U;
    display_context.boot_started = now;
  }
  memset(display_context.sent_frame, 0xFF, sizeof(display_context.sent_frame));
  return 1U;
}

static void SSD1306_FlushOnePage(uint32_t now) {
  uint8_t attempt;

  for (attempt = 0U; attempt < SSD1306_PAGE_COUNT; attempt++) {
    const uint8_t page =
        (uint8_t)((display_context.next_page + attempt) % SSD1306_PAGE_COUNT);
    const uint8_t page_commands[3] = {(uint8_t)(0xB0U | page), 0x00U, 0x10U};

    if (memcmp(display_context.frame[page], display_context.sent_frame[page],
               SSD1306_WIDTH) == 0) {
      continue;
    }
    if ((SSD1306_WriteCommands(page_commands, sizeof(page_commands)) != HAL_OK) ||
        (HAL_I2C_Mem_Write(display_context.i2c,
                           (uint16_t)(SSD1306_I2C_ADDRESS << 1U), 0x40U,
                           I2C_MEMADD_SIZE_8BIT, display_context.frame[page],
                           SSD1306_WIDTH, SSD1306_I2C_TIMEOUT_MS) != HAL_OK)) {
      display_context.online = 0U;
      display_context.retry_deadline = now + SSD1306_RETRY_INTERVAL_MS;
      return;
    }

    memcpy(display_context.sent_frame[page], display_context.frame[page],
           SSD1306_WIDTH);
    display_context.next_page = (uint8_t)((page + 1U) % SSD1306_PAGE_COUNT);
    return;
  }
}

void SSD1306_Status_Init(I2C_HandleTypeDef *i2c_handle) {
  memset(&display_context, 0, sizeof(display_context));
  display_context.i2c = i2c_handle;
}

void SSD1306_Status_Process(void) {
  const uint32_t now = HAL_GetTick();

  if (!display_context.online) {
    if (!SSD1306_DeadlineReached(now, display_context.retry_deadline) ||
        !SSD1306_TryInitialize(now)) {
      return;
    }
  }

  if (SSD1306_DeadlineReached(now, display_context.render_deadline)) {
    if (display_context.boot_active) {
      const uint32_t elapsed = now - display_context.boot_started;
      if (elapsed >= SSD1306_BOOT_DURATION_MS) {
        display_context.boot_active = 0U;
        display_context.boot_completed = 1U;
        SSD1306_RenderStatus();
        display_context.render_deadline = now + SSD1306_RENDER_INTERVAL_MS;
      } else {
        SSD1306_RenderBootAnimation(
            (uint8_t)(elapsed / SSD1306_BOOT_FRAME_INTERVAL_MS));
        display_context.render_deadline =
            now + SSD1306_BOOT_FRAME_INTERVAL_MS;
      }
    } else {
      SSD1306_RenderStatus();
      display_context.render_deadline = now + SSD1306_RENDER_INTERVAL_MS;
    }
  }
  SSD1306_FlushOnePage(now);
}

uint8_t SSD1306_Status_IsOnline(void) { return display_context.online; }

uint8_t SSD1306_Status_TimerStart(void) {
  const uint32_t now = HAL_GetTick();

  if (display_context.timer_running) {
    return 0U;
  }
  display_context.timer_running = 1U;
  display_context.timer_started = now;
  display_context.boot_active = 0U;
  display_context.boot_completed = 1U;
  display_context.render_deadline = now;
  return 1U;
}

uint8_t SSD1306_Status_TimerEnd(uint32_t *elapsed_ms) {
  const uint32_t now = HAL_GetTick();

  if (!display_context.timer_running) {
    return 0U;
  }
  if (elapsed_ms != NULL) {
    *elapsed_ms = now - display_context.timer_started;
  }
  display_context.timer_running = 0U;
  display_context.render_deadline = now;
  return 1U;
}
