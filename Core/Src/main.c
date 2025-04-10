/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "net.h"
#include "planner.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* Структура данных для использования с lwIP сетевым интерфейсом */
extern struct netif gnetif;

/* Буфер входящих по UDP данных из библиотеки net.h */
extern char rxBuf[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM13_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* Прототипы функций для работы с таймером TIM2, настроенным на 1 МГц -> 1 мкс */
uint32_t getMicrosecondsTIM2(void);
void startTimerTIM2(void);
void stopTimerTIM2(void);
void resetTimerTIM2(void);

/** Функция обработки входящих сообщений по UDP
 * 	описание в main.c необходимо для вызова функций другиз библиотек
 */
void udpReceiveHandler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Создание указателя на функцию записи состояния пина микроконтроллера */
static writePinFunction_void_ptr function_pin_1 = (writePinFunction_void_ptr)HAL_GPIO_WritePin;

/* Создание указателя на функцию запсука таймера TIM2 */
static timeFunction_void_ptr function_time_1 = (timeFunction_void_ptr)startTimerTIM2;

/* Создание указателя на функцию остановки таймера TIM2 */
static timeFunction_void_ptr function_time_2 = (timeFunction_void_ptr)stopTimerTIM2;

/* Создание указателя на функцию чтения времени таймера TIM2 */
static timeFunction_uint32_t_ptr function_time_3 = (timeFunction_uint32_t_ptr)getMicrosecondsTIM2;

/* Создание указателя на функцию сброса счетчика таймера TIM2 */
static timeFunction_void_ptr function_time_4 = (timeFunction_void_ptr)resetTimerTIM2;

/* Структуры выводов STEP - DIR - EN для шаговых моторов */
STEPPER_PINS_StructDef stepper1_pins = {(GPIO_StructDef_custom*)STEP1_GPIO_Port, STEP1_Pin, \
										(GPIO_StructDef_custom*)DIR1_GPIO_Port, DIR1_Pin};

STEPPER_PINS_StructDef stepper2_pins = {(GPIO_StructDef_custom*)STEP2_GPIO_Port, STEP2_Pin, \
										(GPIO_StructDef_custom*)DIR2_GPIO_Port, DIR2_Pin};

STEPPER_PINS_StructDef stepper3_pins = {(GPIO_StructDef_custom*)STEP3_GPIO_Port, STEP3_Pin, \
										(GPIO_StructDef_custom*)DIR3_GPIO_Port, DIR3_Pin};

STEPPER_PINS_StructDef stepper4_pins = {(GPIO_StructDef_custom*)STEP4_GPIO_Port, STEP4_Pin, \
										(GPIO_StructDef_custom*)DIR4_GPIO_Port, DIR4_Pin};

STEPPER_PINS_StructDef stepper5_pins = {(GPIO_StructDef_custom*)STEP5_GPIO_Port, STEP5_Pin, \
										(GPIO_StructDef_custom*)DIR5_GPIO_Port, DIR5_Pin};

STEPPER_PINS_StructDef stepper6_pins = {(GPIO_StructDef_custom*)STEP6_GPIO_Port, STEP6_Pin, \
										(GPIO_StructDef_custom*)DIR6_GPIO_Port, DIR6_Pin};

STEPPER_PINS_StructDef stepper7_pins = {(GPIO_StructDef_custom*)STEP7_GPIO_Port, STEP7_Pin, \
										(GPIO_StructDef_custom*)DIR7_GPIO_Port, DIR7_Pin};

STEPPER_PINS_StructDef stepper8_pins = {(GPIO_StructDef_custom*)STEP8_GPIO_Port, STEP8_Pin, \
										(GPIO_StructDef_custom*)DIR8_GPIO_Port, DIR8_Pin};

/* Структуры шаговых моторов */
STEPPER_StructDef stepper1;
STEPPER_StructDef stepper2;

/* Структуры пинов концевых переключателей и датчика нуля драйверов шаговых моторов */
DRIVER_LIMIT_SWITCH_PINS_StructDef driver1_pins = {(GPIO_StructDef_custom*)EXTI_1_LS1_N_GPIO_Port, EXTI_1_LS1_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS1_GPIO_Port, ZERO_POS1_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_0_LS1_P_GPIO_Port, EXTI_0_LS1_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver2_pins = {(GPIO_StructDef_custom*)EXTI_3_LS2_N_GPIO_Port, EXTI_3_LS2_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS2_GPIO_Port, ZERO_POS2_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_2_LS2_P_GPIO_Port, EXTI_2_LS2_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver3_pins = {(GPIO_StructDef_custom*)EXTI_5_LS3_N_GPIO_Port, EXTI_5_LS3_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS3_GPIO_Port, ZERO_POS3_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_4_LS3_P_GPIO_Port, EXTI_4_LS3_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver4_pins = {(GPIO_StructDef_custom*)EXTI_7_LS4_N_GPIO_Port, EXTI_7_LS4_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS4_GPIO_Port, ZERO_POS4_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_6_LS4_P_GPIO_Port, EXTI_6_LS4_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver5_pins = {(GPIO_StructDef_custom*)EXTI_9_LS5_N_GPIO_Port, EXTI_9_LS5_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS5_GPIO_Port, ZERO_POS5_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_8_LS5_P_GPIO_Port, EXTI_8_LS5_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver6_pins = {(GPIO_StructDef_custom*)EXTI_11_LS6_N_GPIO_Port, EXTI_11_LS6_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS6_GPIO_Port, ZERO_POS6_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_10_LS6_P_GPIO_Port, EXTI_10_LS6_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver7_pins = {(GPIO_StructDef_custom*)EXTI_13_LS7_N_GPIO_Port, EXTI_13_LS7_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS7_GPIO_Port, ZERO_POS7_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_12_LS7_P_GPIO_Port, EXTI_12_LS7_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver8_pins = {(GPIO_StructDef_custom*)EXTI_15_LS8_N_GPIO_Port, EXTI_15_LS8_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS8_GPIO_Port, ZERO_POS8_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_14_LS8_P_GPIO_Port, EXTI_14_LS8_P_Pin, NORMALLY_OPEN};

/* Структуры драйверов шаговых моторов */
DRIVER_StructDef driver1;
DRIVER_StructDef driver2;

/* Экземпляр структуры планировщика*/
PLANNER_StructDef planner;

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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Инициализация таймер DWT для одного шага в библиотеке stepper.h */
  DWT_Init();

  /* Инициализация указателей на функции HAL для работы библиотек stepper.h и driver.h */
  stepperFunctionsInit(function_pin_1);
  driverFunctionsInit(function_time_1, function_time_2, function_time_3, function_time_4);

  /* Инициализация шаговых моторов */
  stepperInit(&stepper1, &stepper1_pins);
  stepperInit(&stepper2, &stepper2_pins);

  /* Инициализация драйверов шаговых моторов */
  driverInit(&driver1, &stepper1, &driver1_pins, 5000);
  driverInit(&driver2, &stepper2, &driver2_pins, 5000);

  /* Инициализация планировщика*/
  plannerInit(&planner);

  /* Добавить драйверы в планировщик */
//  addDriver(&planner, &driver1, 0);
//  addDriver(&planner, &driver2, 1);

  /* Инициализация UDP сокета */
  udpSocketInit();

  /* Включение таймера TIM2 */
  startTimerTIM2();

  /* Задание максимальной скорости и ускорения шаговых моторов */
  setAcceleration(&driver1, 1000);
  setMaxSpeed(&driver1, 5000);

  setAcceleration(&driver2, 1000);
  setMaxSpeed(&driver2, 5000);

  /* Тест */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Основные функции управления драйверами */
	  tickDriver(&driver1);
	  tickDriver(&driver2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Ethernet */
	  MX_LWIP_Process();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

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

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ZERO_POS2_Pin|DIR1_Pin|DIR2_Pin|STEP1_Pin
                          |DIR3_Pin|STEP2_Pin|DIR4_Pin|STEP3_Pin
                          |STEP4_Pin|DIR5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ACK_LINE0_Pin|ACK_LINE1_Pin|ACK_LINE2_Pin|STEP6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP7_Pin|STEP8_Pin|DIR6_Pin|DIR7_Pin
                          |DIR8_Pin|STEP5_Pin|SPI3_NSS1_Pin|SPI3_NSS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_STATE_Pin|SPI3_NSS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(READY_GPIO_Port, READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ZERO_POS3_Pin ZERO_POS1_Pin ZERO_POS4_Pin */
  GPIO_InitStruct.Pin = ZERO_POS3_Pin|ZERO_POS1_Pin|ZERO_POS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ZERO_POS2_Pin */
  GPIO_InitStruct.Pin = ZERO_POS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ZERO_POS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_4_LS3_P_Pin EXTI_5_LS3_N_Pin EXTI_1_LS1_N_Pin */
  GPIO_InitStruct.Pin = EXTI_4_LS3_P_Pin|EXTI_5_LS3_N_Pin|EXTI_1_LS1_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ACK_LINE0_Pin ACK_LINE1_Pin ACK_LINE2_Pin */
  GPIO_InitStruct.Pin = ACK_LINE0_Pin|ACK_LINE1_Pin|ACK_LINE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_0_LS1_P_Pin EXTI_2_LS2_P_Pin EXTI_3_LS2_N_Pin EXTI_6_LS4_P_Pin
                           EXTI_8_LS5_P_Pin EXTI_9_LS5_N_Pin */
  GPIO_InitStruct.Pin = EXTI_0_LS1_P_Pin|EXTI_2_LS2_P_Pin|EXTI_3_LS2_N_Pin|EXTI_6_LS4_P_Pin
                          |EXTI_8_LS5_P_Pin|EXTI_9_LS5_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP7_Pin STEP8_Pin DIR6_Pin DIR7_Pin
                           DIR8_Pin STEP5_Pin */
  GPIO_InitStruct.Pin = STEP7_Pin|STEP8_Pin|DIR6_Pin|DIR7_Pin
                          |DIR8_Pin|STEP5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin STEP1_Pin DIR3_Pin
                           STEP2_Pin DIR4_Pin STEP3_Pin STEP4_Pin
                           DIR5_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin|STEP1_Pin|DIR3_Pin
                          |STEP2_Pin|DIR4_Pin|STEP3_Pin|STEP4_Pin
                          |DIR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : EXTI_10_LS6_P_Pin EXTI_11_LS6_N_Pin EXTI_12_LS7_P_Pin EXTI_13_LS7_N_Pin
                           EXTI_14_LS8_P_Pin EXTI_15_LS8_N_Pin EXTI_7_LS4_N_Pin */
  GPIO_InitStruct.Pin = EXTI_10_LS6_P_Pin|EXTI_11_LS6_N_Pin|EXTI_12_LS7_P_Pin|EXTI_13_LS7_N_Pin
                          |EXTI_14_LS8_P_Pin|EXTI_15_LS8_N_Pin|EXTI_7_LS4_N_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP6_Pin */
  GPIO_InitStruct.Pin = STEP6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STEP6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATE_Pin SPI3_NSS2_Pin */
  GPIO_InitStruct.Pin = LED_STATE_Pin|SPI3_NSS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STARTUP_OPTION1_Pin ZERO_POS8_Pin ZERO_POS7_Pin ZERO_POS6_Pin */
  GPIO_InitStruct.Pin = STARTUP_OPTION1_Pin|ZERO_POS8_Pin|ZERO_POS7_Pin|ZERO_POS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : READY_Pin */
  GPIO_InitStruct.Pin = READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(READY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ZERO_POS5_Pin */
  GPIO_InitStruct.Pin = ZERO_POS5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZERO_POS5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_NSS1_Pin SPI3_NSS0_Pin */
  GPIO_InitStruct.Pin = SPI3_NSS1_Pin|SPI3_NSS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/** Запуск таймера для отсчета времени в микросекундах
 * 	Используется таймер TIM2
 */
void startTimerTIM2(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
}

/** Остановка таймера для отсчета времени в микросекундах
 * 	Используется таймер TIM2
 */
void stopTimerTIM2(void)
{
	HAL_TIM_Base_Stop_IT(&htim2);
}

/** Функция сброса счетчика таймера в 0
 * 	Используется таймер TIM2
 */
void resetTimerTIM2(void)
{
	TIM2->CNT = 0;
}

/** Функция возвращает время в микросекундах с момента включения устройства
 * Используется таймер TIM2
 * Частота работы таймера 1МГц
 * 1 тик таймера = 1 мкс
 */
uint32_t getMicrosecondsTIM2(void)
{
	return TIM2->CNT;
}

/**
 *
 */
void udpReceiveHandler(void)
{
	char data[256];

	int target_pos = strtol(rxBuf, NULL, 10);
	memset(rxBuf, 0, 128);

	setTarget(&driver1, target_pos);
	setTarget(&driver2, target_pos);

	sprintf(data, "target = %d;\n", target_pos);
	udpClientSend(data);
}

/** Функция проверки свзяи с пинами STEP - DIR
 * 	К выводам подключены светодиоды!
 */
void testStepDirPin()
{
	/* Пины STEP */
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

	/* Пины DIR */
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);

	HAL_Delay(500);
}

/* USER CODE END 4 */

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
