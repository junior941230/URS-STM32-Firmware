/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/*
 * MSP 層只負責 MCU peripheral 的底層資源：clock、GPIO alternate function、NVIC。
 * 協定與 Motor CAN state machine 不放在這裡。邏輯 CAN1 使用 FDCAN3，邏輯 CAN2
 * 使用 FDCAN2；兩個 instance 共用 FDCAN kernel clock，因此以計數器管理生命週期。
 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* FDCAN2 與 FDCAN3 共用周邊 clock，使用計數器避免其中一路誤關閉 clock。 */
static uint32_t HAL_RCC_FDCAN_CLK_ENABLED = 0U;

/**
* @brief 初始化 FDCAN 的 MCU 支援套件資源
* @param hfdcan FDCAN handle 指標
* @retval 無
*/
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if ((hfdcan->Instance == FDCAN2) || (hfdcan->Instance == FDCAN3))
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* 只在第一個 instance 初始化時開 clock，避免重複開關共用資源。 */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if (HAL_RCC_FDCAN_CLK_ENABLED == 1U)
    {
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (hfdcan->Instance == FDCAN2)
    {
      /* 硬體 CAN2：PB5 為 RX，PB6 為 TX。 */
      GPIO_InitStruct.Pin = CAN2_RX_Pin | CAN2_TX_Pin;
      GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* CAN IRQ priority 1，僅低於 priority 0 的 EMS EXTI。 */
      HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
    else
    {
      /* 硬體 CAN1：PB3 為 RX，PB4 為 TX；PB3 因此不能再作為 SWO。 */
      GPIO_InitStruct.Pin = CAN1_RX_Pin | CAN1_TX_Pin;
      GPIO_InitStruct.Alternate = GPIO_AF11_FDCAN3;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 1, 0);
      HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }
  }
}

/**
* @brief 釋放 FDCAN 使用的 MCU 支援套件資源
* @param hfdcan FDCAN handle 指標
* @retval 無
*/
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  if (hfdcan->Instance == FDCAN2)
  {
    HAL_GPIO_DeInit(GPIOB, CAN2_RX_Pin | CAN2_TX_Pin);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  }
  else if (hfdcan->Instance == FDCAN3)
  {
    HAL_GPIO_DeInit(GPIOB, CAN1_RX_Pin | CAN1_TX_Pin);
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
  }
  else
  {
    return;
  }

  if (HAL_RCC_FDCAN_CLK_ENABLED > 0U)
  {
    HAL_RCC_FDCAN_CLK_ENABLED--;
  }
  if (HAL_RCC_FDCAN_CLK_ENABLED == 0U)
  {
    __HAL_RCC_FDCAN_CLK_DISABLE();
  }
}

/**
* @brief 初始化 I2C 的 MCU 支援套件資源
* @param hi2c I2C handle 指標
* @retval 無
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  if (hi2c->Instance == I2C1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* PCB2 已有4.7kΩ外部上拉，所以 MCU 不啟用內部 pull-up。 */
    GPIO_InitStruct.Pin = OLED_SCL_Pin | OLED_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

/**
* @brief 釋放 I2C 使用的 MCU 支援套件資源
* @param hi2c I2C handle 指標
* @retval 無
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, OLED_SCL_Pin | OLED_SDA_Pin);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
