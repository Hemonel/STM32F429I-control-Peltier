/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"pantalla.h"
#include"industria3_240x320.h"
#include "stm32f4xx_hal.h"
//#include"imagenAlerta1_70x70.h"

#include "mas.h"
#include "menos.h"
#include"Consolas9x18Amarillo.h"
#include "botonReset60x60.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//void funcionpruebaboton();
void funcionBoton1();
void funcionBoton2();
void funcionBoton3();
void funcionBoton4();
void funcionBoton5();
void funcionBoton6();
void funcionBoton7();
void funcionBoton8();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tareaPantalla */
osThreadId_t tareaPantallaHandle;
const osThreadAttr_t tareaPantalla_attributes = {
  .name = "tareaPantalla",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for temporizador */
osTimerId_t temporizadorHandle;
const osTimerAttr_t temporizador_attributes = {
  .name = "temporizador"
};
/* Definitions for semaforoContador */
osSemaphoreId_t semaforoContadorHandle;
const osSemaphoreAttr_t semaforoContador_attributes = {
  .name = "semaforoContador"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void fTareaPantalla(void *argument);
void temporizadorTick(void *argument);

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
  MX_CRC_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaforoContador */
  semaforoContadorHandle = osSemaphoreNew(1, 1, &semaforoContador_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of temporizador */
  temporizadorHandle = osTimerNew(temporizadorTick, osTimerPeriodic, NULL, &temporizador_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of tareaPantalla */
  tareaPantallaHandle = osThreadNew(fTareaPantalla, NULL, &tareaPantalla_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 9;
  hltdc.Init.VerticalSync = 1;
  hltdc.Init.AccumulatedHBP = 29;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 269;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 327;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Boton botonReset;  // Estructura donde se guarda información para el manejo del botón
uint32_t contador = 0;
uint8_t numResets = 0;
TsBoton botonReset1,botonReset2,botonReset3,botonReset4,botonReset5,botonReset6,botonReset7,botonReset8;
double Kp=2.3,Kd=0.1,Ki=0.2,Consigna=40.0,errorAnt=0.0,actua,sumaError=0.0;
int Ts=3,cntTs=10,cntTimer=10,espera=0;
int ciclo=50,periodo=100,cntTime=0,medida;
JuegoCaracteresAlpha  juegoConsolas9x18;
PantallaLCD  pantallaLCD;
void double2float(double dato, char cadena[])
{	int entera,real;

	entera=(int)dato;
	real=(int)((dato-entera)*10);
	sprintf(cadena, "%s%d.%d",cadena,entera,real);
	return;
}
void funcionBoton1() {  // Función a ejecutar cuando se pulse el botón
	double Consignaaux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	Consignaaux=Consigna;
    if (Consigna>0)
		Consigna-=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Cnsg=");
    double2float(Consignaaux,cadena);
    dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);

}
void funcionBoton2() {  // Función a ejecutar cuando se pulse el botón
	double KpAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KpAux=Kp;
    if (Kp>0)
    {
    	Kp-=0.1;  // Cuando se pulsa el botón, se resetea el contador
    }
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Kp=");
    double2float(KpAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);

}
void funcionBoton3() {  // Función a ejecutar cuando se pulse el botón
	double KdAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KdAux=Kd;
	if (Kd>0)
		Kd-=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Kd=");
    double2float(KdAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
}

void funcionBoton4() {  // Función a ejecutar cuando se pulse el botón
	double KiAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KiAux=Ki;
    if(Ki>0)
    {
    	Ki-=0.1;  // Cuando se pulsa el botón, se resetea el contador
    }
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Ki=");
    double2float(KiAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
}
void funcionBoton5() {  // Función a ejecutar cuando se pulse el botón
	double Consignaaux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	Consignaaux=Consigna;
	Consigna+=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Cnsg=");
    double2float(Consignaaux,cadena);
    dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);

}
void funcionBoton6() {  // Función a ejecutar cuando se pulse el botón
	double KpAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KpAux=Kp;
    Kp+=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Kp=");
    double2float(KpAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);


}
void funcionBoton7() {  // Función a ejecutar cuando se pulse el botón
	double KdAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KdAux=Kd;
    Kd+=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Kd=");
    double2float(KdAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
}


void funcionBoton8() {  // Función a ejecutar cuando se pulse el botón
	double KiAux;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	KiAux=Ki;
    Ki+=0.1;  // Cuando se pulsa el botón, se resetea el contador
	osSemaphoreRelease(semaforoContadorHandle);
    char cadena[100];
    sprintf(cadena, "Ki=");
    double2float(KiAux,cadena);
    dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
    dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
    intercambiaBuffersLCD(&pantallaLCD);
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
#define COLOR_FONDO_PANTALLA 0xFF000030  // Color azul para el fondo de la pantalla
#define COLOR_TEXTO 0xFFFFFF00  // Color amarillo para el texto
#define COLOR_DIBUJO 0xFFFF0000  // Color rojo para dibujar

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */


  	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
  	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
  	//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET); //siempre calor Probar otros valores

  inicializaPantalla2Buffers(1, COLOR_FONDO_PANTALLA, industria3_240x320, &pantallaLCD);

  inicializaBoton(5, 10, 60, 60, menos,funcionBoton1, 1, 1, &botonReset1, &pantallaLCD);
  inicializaBoton(5, 90, 60, 60, menos,funcionBoton2, 1, 1, &botonReset2, &pantallaLCD);
  inicializaBoton(5, 170, 60, 60, menos,funcionBoton3, 1, 1, &botonReset3, &pantallaLCD);
  inicializaBoton(5, 250, 60, 60, menos,funcionBoton4, 1, 1, &botonReset4, &pantallaLCD);

  inicializaBoton(170, 10, 60, 60, mas,funcionBoton5, 1, 1, &botonReset5, &pantallaLCD);
  inicializaBoton(170, 90, 60, 60, mas,funcionBoton6, 1, 1, &botonReset6, &pantallaLCD);
  inicializaBoton(170, 170, 60, 60, mas,funcionBoton7, 1, 1, &botonReset7, &pantallaLCD);
  inicializaBoton(170, 250, 60, 60, mas,funcionBoton8, 1, 1, &botonReset8, &pantallaLCD);

  inicializaJuegoCaracteresAlpha(9, 18, 0, Consolas9x18Amarillo, &juegoConsolas9x18);
  char cadena[100];
  sprintf(cadena, "Cnsg=");
  double2float(Consigna,cadena);
  dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  dibujaCadenaCaracteresAlpha(70, 30, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  sprintf(cadena, "Kp=");
  double2float(Kp,cadena);
  dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  dibujaCadenaCaracteresAlpha(70, 110, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  sprintf(cadena, "Kd=");
  double2float(Kd,cadena);
  dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  //sprintf(cadena, "Kd=");
  dibujaCadenaCaracteresAlpha(70, 190, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  sprintf(cadena, "Ki=");
  double2float(Ki,cadena);
  dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0,100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  dibujaCadenaCaracteresAlpha(70, 270, cadena, &juegoConsolas9x18, 0, 100, 1, &pantallaLCD);
  intercambiaBuffersLCD(&pantallaLCD);
  //sprintf(cadena, "Ki=");
  osTimerStart(temporizadorHandle,100);
  while(1) {  // Repite contínuamente //.



        uint16_t xClick, yClick;
        int pulsada = pantallaPulsada( &xClick, &yClick);
        atiendeBoton(xClick, yClick, pulsada, &botonReset1);
        atiendeBoton(xClick, yClick, pulsada, &botonReset2);
        atiendeBoton(xClick, yClick, pulsada, &botonReset3);
        atiendeBoton(xClick, yClick, pulsada, &botonReset4);
        atiendeBoton(xClick, yClick, pulsada, &botonReset5);
        atiendeBoton(xClick, yClick, pulsada, &botonReset6);
        atiendeBoton(xClick, yClick, pulsada, &botonReset7);
        atiendeBoton(xClick, yClick, pulsada, &botonReset8);
        osDelay(100);

  }

for(;;)
{
  osDelay(1);
}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fTareaPantalla */
/**
* @brief Function implementing the tareaPantalla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTareaPantalla */
void fTareaPantalla(void *argument)
{
  /* USER CODE BEGIN fTareaPantalla */
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END fTareaPantalla */
}

/* temporizadorTick function */
void temporizadorTick(void *argument)
{
  /* USER CODE BEGIN temporizadorTick */

cntTime++;
if (cntTime>ciclo) //la placa peltier solo trabaja parte del ciclo
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
else
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
if (cntTime==periodo)
	cntTime=0;
//-----------------------------------------------------------------------------

cntTs--;
if(cntTs==0)
	{
	cntTs=Ts;
	osSemaphoreAcquire(semaforoContadorHandle,osWaitForever);
	osSemaphoreRelease(semaforoContadorHandle);
	HAL_ADC_Start(&hadc1); //inicio conversión
	HAL_ADC_PollForConversion(&hadc1,100); // Espera fin de conversión con un timeout
	medida=HAL_ADC_GetValue(&hadc1); // Recoge los 12 bits resultado de la conversión
	medida=medida*330/4096; //medida*(Vmax/sensibilidad sesnor)/2^nbits
	HAL_ADC_Stop(&hadc1); //fin conversión

	error=Consigna-medida; //diferencia entre la temperatura consigna y la real
	sumaError+=error;
	actua=(Kp*error)+(Ki*Ts*0.1*sumaError)+(Kd*Ts*0.1*(error-errorAnt)); //PID
	errorAnt=error;
	double abs=actua;
	if (actua>1){ //si está más caliente
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
	}
	else if (actua<-1){ //si está más frío
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	}
	else { //si está a la temperatura correcta
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
	}
	if (actua>100) //si la actuación es mayor de 100 calentará el 100% del ciclo
		actua=100;
	if (actua <-100) //si la actuación es menor de -100 enfriará el 100% del ciclo
		actua=-100;
	if (actua<0) //si la actuación es negativa la vuelve positiva
		actua=-actua;
	ciclo=(int)actua;
	}

//-------------------------------------------------------------
cntTimer--;
if (cntTimer==0)
	{
	char cadena[20];
	cntTimer=(cntTs)+espera;
	sprintf(cadena,"%d %d %d %d %d %d\r\n",(int)Consigna,(int)actua,(int)medida,(int)Kp,(int)Kd,(int)Ki);

	if (HAL_UART_Transmit(&huart1,(uint8_t *) cadena,strlen(cadena),5000))
		espera+=5;
	}


  /* USER CODE END temporizadorTick */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

#ifdef  USE_FULL_ASSERT
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
