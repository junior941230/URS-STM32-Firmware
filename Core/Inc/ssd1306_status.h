#ifndef SSD1306_STATUS_H
#define SSD1306_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

/* SSD1306 常見 7-bit I2C 位址；若模組使用 0x3D，可由編譯設定覆寫。 */
#ifndef SSD1306_I2C_ADDRESS
#define SSD1306_I2C_ADDRESS 0x3CU
#endif

/** @brief 綁定 SSD1306 使用的 I2C；實際偵測與初始化由 Process 觸發。 */
void SSD1306_Status_Init(I2C_HandleTypeDef *i2c_handle);

/** @brief 更新狀態畫面；每次最多傳送一個 128-byte page。 */
void SSD1306_Status_Process(void);

/** @brief 回傳最近一次 SSD1306 通訊是否成功。 */
uint8_t SSD1306_Status_IsOnline(void);

/** @brief 從 0 開始計時；已在計時時回傳 0。 */
uint8_t SSD1306_Status_TimerStart(void);

/** @brief 結束計時並輸出經過毫秒；未在計時時回傳 0。 */
uint8_t SSD1306_Status_TimerEnd(uint32_t *elapsed_ms);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_STATUS_H */
