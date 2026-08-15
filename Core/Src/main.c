/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_can.h"
#include "usb_command.h"
#include "usb_device.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* 硬體 CAN2 使用 FDCAN2；硬體 CAN1 使用 FDCAN3。 */
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* OLED 顯示器使用 I2C1。 */
I2C_HandleTypeDef hi2c1;

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
/*
 * EMS 狀態由 EXTI callback 更新，主迴圈與 USB callback 讀取，所以必須 volatile。
 * HIGH 代表立即停止；HIGH->LOW 後仍保持 command block，直到主迴圈清除 MotorCAN
 * state、CAN RX queue 與 USB RX parser，避免 EMS 期間殘留的命令在釋放後執行。
 */
static volatile uint8_t ems_stop_active;
static volatile uint8_t ems_release_cleanup_pending;
/* 每次 EMS 釋放只清一次 USB RX，之後仍允許 PING/HELP/STATUS。 */
static volatile uint8_t ems_usb_cleanup_pending;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_I2C1_Init(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /*
   * 專案刻意用邏輯 bus 編號隔離 MCU peripheral 名稱：
   *   bus 1 -> FDCAN3 -> PCB CAN1 接頭
   *   bus 2 -> FDCAN2 -> PCB CAN2 接頭
   * 後續 USB 命令只看到 bus 1/2，不需要知道 STM32 instance 編號。
   */
  if (MotorCAN_Init(&hfdcan3, &hfdcan2) != MOTOR_CAN_STATUS_OK)
  {
    Error_Handler();
  }

  /* 先清空 command queue，再啟動 USB CDC，避免列舉 callback 讀到未初始化索引。 */
  USB_Command_Init();
  MX_USB_Device_Init();

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on led ---- */
  BSP_LED_On(LED_GREEN);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* -- Sample board code for User push-button in interrupt mode ---- */
    if (BspButtonState == BUTTON_PRESSED)
    {
      /* Update button state */
      BspButtonState = BUTTON_RELEASED;
      /* -- Sample board code to toggle led ---- */
      BSP_LED_Toggle(LED_GREEN);

      /* ..... Perform your action ..... */
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * EMS 釋放採兩階段 handshake：ISR 只標記 cleanup pending；主迴圈才呼叫
     * 可能操作 FDCAN 與 queue 的清理函式。兩邊都清完後才解除 command block。
     */
    if (EMS_IsReleaseCleanupPending())
    {
      if (ems_usb_cleanup_pending)
      {
        USB_Command_ClearPendingCommands();
        ems_usb_cleanup_pending = 0U;
      }
      if (MotorCAN_ClearPendingCommands())
      {
        EMS_CompleteReleaseCleanup();
      }
    }
    /*
     * 先處理 CAN/EMS，再解析 USB。若本輪剛收到 EMS，MotorCAN 可優先停止動作；
     * USB command 層則依 EMS_AreCommandsBlocked() 決定是否接受下一條命令。
     */
    MotorCAN_Process();
    USB_Command_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 初始化 FDCAN2，對應硬體上的 CAN2
  * @param 無
  * @retval 無
  */
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  /* 170 MHz / (10 × (1 + 13 + 3)) = 1 Mbit/s，sample point 為 82.35%。 */
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  /* Classic CAN 不使用 data phase；HAL 仍要求填入有效設定。 */
  hfdcan2.Init.DataPrescaler = 10;
  hfdcan2.Init.DataSyncJumpWidth = 3;
  hfdcan2.Init.DataTimeSeg1 = 13;
  hfdcan2.Init.DataTimeSeg2 = 3;
  hfdcan2.Init.StdFiltersNbr = 8;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 初始化 FDCAN3，對應硬體上的 CAN1
  * @param 無
  * @retval 無
  */
static void MX_FDCAN3_Init(void)
{
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = ENABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  /* 170 MHz / (10 × (1 + 13 + 3)) = 1 Mbit/s，sample point 為 82.35%。 */
  hfdcan3.Init.NominalPrescaler = 10;
  hfdcan3.Init.NominalSyncJumpWidth = 3;
  hfdcan3.Init.NominalTimeSeg1 = 13;
  hfdcan3.Init.NominalTimeSeg2 = 3;
  /* Classic CAN 不使用 data phase；HAL 仍要求填入有效設定。 */
  hfdcan3.Init.DataPrescaler = 10;
  hfdcan3.Init.DataSyncJumpWidth = 3;
  hfdcan3.Init.DataTimeSeg1 = 13;
  hfdcan3.Init.DataTimeSeg2 = 3;
  hfdcan3.Init.StdFiltersNbr = 8;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief 初始化供 OLED 使用的 I2C1
  * @param 無
  * @retval 無
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  /* Fast Mode 400 kHz；I2C kernel clock 170 MHz，tr=100 ns，tf=10 ns。 */
  hi2c1.Init.Timing = 0x00C0216C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* EMS_SW 由外部電路偏壓：常閉迴路正常時為 LOW，斷路或停止時為 HIGH。 */
  GPIO_InitStruct.Pin = EMS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EMS_SW_GPIO_Port, &GPIO_InitStruct);

  /* EMS 是應用程式中優先權最高的外部輸入。 */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*
   * 上電時直接採用 EMS 腳位現況。若當下為 HIGH，USB 仍可完成列舉與傳送
   * PING/HELP/STATUS；其餘命令由 parser 回覆 EMS_ACTIVE。
   */
  ems_stop_active =
    (HAL_GPIO_ReadPin(EMS_SW_GPIO_Port, EMS_SW_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  ems_release_cleanup_pending = 0U;
  ems_usb_cleanup_pending = 0U;
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief 取得 EMS 目前是否處於停止狀態
  * @retval 1 表示 EMS 仍為 HIGH；0 表示 EMS 已釋放
  */
uint8_t EMS_IsStopActive(void)
{
  return ems_stop_active;
}

/**
  * @brief 判斷是否暫停接受新指令
  * @retval 1 表示 EMS 啟動中或釋放清理尚未完成
  */
uint8_t EMS_AreCommandsBlocked(void)
{
  /* 釋放後的短暫 cleanup window 也必須阻擋命令，避免 ISR 與主迴圈競態。 */
  return (ems_stop_active || ems_release_cleanup_pending) ? 1U : 0U;
}

/**
  * @brief 判斷 EMS 釋放後是否尚未清除未完成指令
  * @retval 1 表示主迴圈必須執行清理
  */
uint8_t EMS_IsReleaseCleanupPending(void)
{
  return ems_release_cleanup_pending;
}

/**
  * @brief 通知 EMS 釋放清理已完成，可以接受新指令
  * @retval 無
  */
void EMS_CompleteReleaseCleanup(void)
{
  ems_release_cleanup_pending = 0U;
  ems_usb_cleanup_pending = 0U;
}

/**
  * @brief GPIO EXTI callback；同步 EMS 啟動與釋放狀態
  * @param GPIO_Pin 觸發中斷的 GPIO 腳位
  * @retval 無
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == EMS_SW_Pin)
  {
    /*
     * 使用同一個 rising/falling callback 重新讀取 pin，而不依賴 edge 類型：
     * HIGH 立即封鎖控制；LOW 先要求主迴圈清理，清理完成後才恢復命令。
     */
    if (HAL_GPIO_ReadPin(EMS_SW_GPIO_Port, EMS_SW_Pin) == GPIO_PIN_SET)
    {
      ems_stop_active = 1U;
      ems_release_cleanup_pending = 0U;
      ems_usb_cleanup_pending = 0U;
    }
    else
    {
      ems_stop_active = 0U;
      ems_release_cleanup_pending = 1U;
      ems_usb_cleanup_pending = 1U;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
