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
#include "libs.h"

#include "stdbool.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* Импортированная структура данных для использования сетевого интерфейса UDP из библиотеки lwip.c */
extern struct netif gnetif;

/* Импортированный счетчик принятых сообщений по UDP интерфейсу */
extern int32_t counter;

/* Импортированный буфер входящих по UDP данных из библиотеки net.h */
extern char rxUdpCharBuf[128];

/* Импортированный FIFO буфер сетевого интерфейса UDP из библиотеки het.h */
extern FIFO_StructDef fifoNetBuf;

/* Импортированная структура UDP команд для отладки из библиотеки het.h */
extern UDP_COMMANDS_StructDef udpcommands;

/* Импортированный FIFO буфер G - команд из библиотеки gcode.h */
extern FIFO_CHAR_StructDef fifoGcodeBuf;

/* Импортированный FIFO буфер шагов интерполятора */
extern FIFO_StructDef fifoBufSteps;

/*Импортированные буферы предзаписанного G - кода */

/* 5 строк, траектория - "пример" */
extern char* GcodeBufferExample[5];

/* 4 строки, траектория - "окружность" */
extern char* GcodeBufferCircle[4];

/* 19 строк, траектория - "тестовая деталь" */
extern char* GcodeBuffer1[19];

/* 119 строк, траектория - "жираф" */
extern char* GcodeBuffer2[119];

/* 129 строк, траектория - "медуза" */
extern char* GcodeBuffer3[129];

/* 129 строк, траектория - "крокодил" */
extern char* GcodeBuffer4[243];
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
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* Прототипы функций для работы с таймером TIM2, настроенным на 1 МГц -> 1 мкс */
uint32_t getMicrosecondsTIM2(void);
void startTimerTIM2(void);
void stopTimerTIM2(void);
void resetTimerTIM2(void);

/* Прототипы функций для работы с таймером TIM5, настроенным на 1 МГц -> 1 мкс */
void startTimerTIM1(void);

/** Функция обработки входящих сообщений по UDP
 * 	описание в main.c необходимо для вызова функций других библиотек
 */
void udpReceiveHandlerEcho(void);

/** Тестовая функция для проверки пинов STEP - DIR
 */
void testStepDirPin(void);

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
STEPPER_PINS_StructDef stepper0_pins = {(GPIO_StructDef_custom*)STEP1_GPIO_Port, STEP1_Pin, \
										(GPIO_StructDef_custom*)DIR1_GPIO_Port, DIR1_Pin};

STEPPER_PINS_StructDef stepper1_pins = {(GPIO_StructDef_custom*)STEP2_GPIO_Port, STEP2_Pin, \
										(GPIO_StructDef_custom*)DIR2_GPIO_Port, DIR2_Pin};

STEPPER_PINS_StructDef stepper2_pins = {(GPIO_StructDef_custom*)STEP3_GPIO_Port, STEP3_Pin, \
										(GPIO_StructDef_custom*)DIR3_GPIO_Port, DIR3_Pin};

STEPPER_PINS_StructDef stepper3_pins = {(GPIO_StructDef_custom*)STEP4_GPIO_Port, STEP4_Pin, \
										(GPIO_StructDef_custom*)DIR4_GPIO_Port, DIR4_Pin};

STEPPER_PINS_StructDef stepper4_pins = {(GPIO_StructDef_custom*)STEP5_GPIO_Port, STEP5_Pin, \
										(GPIO_StructDef_custom*)DIR5_GPIO_Port, DIR5_Pin};

STEPPER_PINS_StructDef stepper5_pins = {(GPIO_StructDef_custom*)STEP6_GPIO_Port, STEP6_Pin, \
										(GPIO_StructDef_custom*)DIR6_GPIO_Port, DIR6_Pin};

STEPPER_PINS_StructDef stepper6_pins = {(GPIO_StructDef_custom*)STEP7_GPIO_Port, STEP7_Pin, \
										(GPIO_StructDef_custom*)DIR7_GPIO_Port, DIR7_Pin};

STEPPER_PINS_StructDef stepper7_pins = {(GPIO_StructDef_custom*)STEP8_GPIO_Port, STEP8_Pin, \
										(GPIO_StructDef_custom*)DIR8_GPIO_Port, DIR8_Pin};

/* Структуры пинов концевых переключателей и датчика нуля драйверов шаговых моторов */
DRIVER_LIMIT_SWITCH_PINS_StructDef driver0_pins = {(GPIO_StructDef_custom*)EXTI_1_LS1_N_GPIO_Port, EXTI_1_LS1_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS1_GPIO_Port, ZERO_POS1_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_0_LS1_P_GPIO_Port, EXTI_0_LS1_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver1_pins = {(GPIO_StructDef_custom*)EXTI_3_LS2_N_GPIO_Port, EXTI_3_LS2_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS2_GPIO_Port, ZERO_POS2_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_2_LS2_P_GPIO_Port, EXTI_2_LS2_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver2_pins = {(GPIO_StructDef_custom*)EXTI_5_LS3_N_GPIO_Port, EXTI_5_LS3_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS3_GPIO_Port, ZERO_POS3_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_4_LS3_P_GPIO_Port, EXTI_4_LS3_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver3_pins = {(GPIO_StructDef_custom*)EXTI_7_LS4_N_GPIO_Port, EXTI_7_LS4_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS4_GPIO_Port, ZERO_POS4_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_6_LS4_P_GPIO_Port, EXTI_6_LS4_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver4_pins = {(GPIO_StructDef_custom*)EXTI_9_LS5_N_GPIO_Port, EXTI_9_LS5_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS5_GPIO_Port, ZERO_POS5_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_8_LS5_P_GPIO_Port, EXTI_8_LS5_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver5_pins = {(GPIO_StructDef_custom*)EXTI_11_LS6_N_GPIO_Port, EXTI_11_LS6_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS6_GPIO_Port, ZERO_POS6_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_10_LS6_P_GPIO_Port, EXTI_10_LS6_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver6_pins = {(GPIO_StructDef_custom*)EXTI_13_LS7_N_GPIO_Port, EXTI_13_LS7_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS7_GPIO_Port, ZERO_POS7_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_12_LS7_P_GPIO_Port, EXTI_12_LS7_P_Pin, NORMALLY_OPEN};

DRIVER_LIMIT_SWITCH_PINS_StructDef driver7_pins = {(GPIO_StructDef_custom*)EXTI_15_LS8_N_GPIO_Port, EXTI_15_LS8_N_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)ZERO_POS8_GPIO_Port, ZERO_POS8_Pin, NORMALLY_OPEN, \
												   (GPIO_StructDef_custom*)EXTI_14_LS8_P_GPIO_Port, EXTI_14_LS8_P_Pin, NORMALLY_OPEN};

/* Структура пинов лазера */
LASER_PINS_StructDef laser_pins = {(GPIO_StructDef_custom*)LED_STATE_GPIO_Port, LED_STATE_Pin}; //< Для теста используется пины светодиода на плате контроллера

/* Структуры шаговых моторов */
STEPPER_StructDef stepper[8];

/* Структуры драйверов шаговых моторов */
DRIVER_StructDef driver[8];

/* Экземпляр структуры планировщика*/
PLANNER_StructDef planner;

/* Экземпляр структуры лазера */
LASER_StructDef laser;

/* Экземпляр структуры обработчика g - кода */
HANDLER_GCODE_StructDef ghandler;

/* Flag включения обработки */
bool _runFlag = false;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* ----------------------------------------------- Инициализация -------------------------------------------- */

  /* Инициализация таймер DWT для одного шага в библиотеке stepper.h */
  DWT_Init();

  /* Инициализация UDP сокета */
  UDP_Init();

  /* Инициализация указателей на функции HAL для работы библиотек pin.h и driver.h */
  pinFunctionsInit(function_pin_1);
  driverFunctionsInit(function_time_1, function_time_2, function_time_3, function_time_4);

  /* Инициализация шаговых моторов */
  stepperInit(&stepper[0], &stepper0_pins);
  stepperInit(&stepper[1], &stepper1_pins);

  /* Инициализация драйверов шаговых моторов */
  driverInit(&driver[0], &stepper[0], &driver0_pins, 800, LINEAR, 'X');
  driverInit(&driver[1], &stepper[1], &driver1_pins, 800, LINEAR, 'Y');

  /* Инициализация планировщика*/
  plannerInit(&planner);
  plannerFunctionsInit(function_time_3);

  /* Инициализация обработчика g - команд*/
  handlerGcodeInit(&ghandler);

  /* Инициализация лазера */
  laserInit(&laser, &laser_pins);

  /* ----------------------------------------------- Инициализация -------------------------------------------- */

  /* Добавить драйверы в планировщик */
  addDriver(&planner, &driver[0], 0);
  addDriver(&planner, &driver[1], 1);

  /* Запуск таймера TIM2 */
  startTimerTIM2();

  /* Запуск таймера TIM1 */
//  startTimerTIM1();

  /* Включить драйверы моторов */
  for(uint8_t i = 0; i < AXES; i ++)
  {
	  enableDriver(&driver[i]);
  }

  /* Тест */

  driver[0].stepper->_globDir = true;
  driver[1].stepper->_globDir = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Вызов тестовой функции для проверки плат интерфейса шагового мотора */
//	  testStepDirPin();

	  /* Основные функции управления драйверами */
	  for(uint8_t i = 0; i < AXES; i ++)
	  {
		  tickDriver(&driver[i]);
	  }

	  /* Тикер обработчика g - кода */
	  tickGcodeHandler(&ghandler);

	  /* Тикер осевого планировщика */
	  if(_runFlag == true)
	  {
		  if (availableForRead(&fifoBufSteps) == FIFO_OK)
		  {
			  /* Тикер планировщика многоосевого движения */
			  tickPlanner(&planner);
		  }

		  if(availableForReadChar(&fifoGcodeBuf) == FIFO_EMPTY && availableForRead(&fifoBufSteps) == FIFO_EMPTY)
		  {
			  handlerEndState(&ghandler);
			  planner._workState = PLANNER_END;
			  _runFlag = false;
		  }
	  }

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
  htim1.Init.Prescaler = 1680 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000 - 1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

/** Запуск таймера для интерполятора
 * 	Используется таймер TIM3
 */
void startTimerTIM1(void)
{
	HAL_TIM_Base_Start_IT(&htim1);
}


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
void udpReceiveHandlerEcho(void)
{
	/* Буфер данных для отправки ответного сообщения по UDP */
	char data[256];

	/* Тестовые функции переинициализации буфера G - кода */
	if(strcmp(rxUdpCharBuf, udpcommands.test1) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBuffer1, 256);
		fifoGcodeBuf.head = 19;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Test - 1 has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strcmp(rxUdpCharBuf, udpcommands.test2) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBuffer2, 256);
		fifoGcodeBuf.head = 119;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Test - 2 has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strcmp(rxUdpCharBuf, udpcommands.test3) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBuffer3, 256);
		fifoGcodeBuf.head = 129;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Test - 3 has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strcmp(rxUdpCharBuf, udpcommands.test4) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBuffer4, 256);
		fifoGcodeBuf.head = 243;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Test - 4 has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strcmp(rxUdpCharBuf, udpcommands.circle) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBufferCircle, 256);
		fifoGcodeBuf.head = 4;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Circle has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strcmp(rxUdpCharBuf, udpcommands.example) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is running, unable to complete test!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		fifoClear(&fifoBufSteps);

		/* Переинициализация FIFO буфера G - команд */
		fifoInitChar(&fifoGcodeBuf, GcodeBufferExample, 256);
		fifoGcodeBuf.head = 5;

		ghandler._workState = HANDLER_GCODE_READY;

		sprintf(data, "%ld - STM32: Example has been initialized!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Запуск процесса работы станка */
	if(strcmp(rxUdpCharBuf, udpcommands.start) == 0)
	{
		/* Проверка уже запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is already running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		if(availableForRead(&fifoBufSteps) == FIFO_EMPTY)
		{
			sprintf(data, "%ld - STM32: G-code program is empty!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		calculatePlannerInitialParam(&planner);

		/* Флаг запуска процесса работы */
		_runFlag = true;

		sprintf(data, "%ld - STM32: The process has been started!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Запуск приостановки работы станка */
	if(strcmp(rxUdpCharBuf, udpcommands.pause) == 0)
	{
		/* Проверка запущенного процесса работы станка */
		if(_runFlag == false)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is not running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		/* Здесь должна быть функция паузы станка */
		pausePlanner(&planner);

		sprintf(data, "%ld - STM32: The process has been paused!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Запуск возобновления процесса работы станка */
	if(strcmp(rxUdpCharBuf, udpcommands.resume) == 0)
	{
		/* Проверка запущенного процесса работы станка */
		if(_runFlag == false)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is not running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		/* Здесь должна быть функция возобновления станка */
		resumePlanner(&planner);

		sprintf(data, "%ld - STM32: The process has been resumed!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Запуск резкой остановки процесса работы станка с возможностью продолжения процесса */
	if(strcmp(rxUdpCharBuf, udpcommands.stop) == 0)
	{
		/* Проверка запущенного процесса работы станка */
		if(_runFlag == false)
		{
			sprintf(data, "%ld - STM32: Attention! The machine is not running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		/* Здесь должна быть функция остановки станка */
		stopPlanner(&planner);

		sprintf(data, "%ld - STM32: The process has been stopped!\n", counter);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setPlannerAcceleration, strlen(udpcommands.setPlannerAcceleration)) == 0)
	{
		/* Проверка запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Error changing! The machine is running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		float acceleration = atof(strchr(rxUdpCharBuf, ' ') + 1);
		setPlannerAcceleration(&planner, acceleration);

		sprintf(data, "%ld - STM32: Planner acceleration = %ld (steps/sec^2);\n", counter, planner._accel);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setPlannerMaxSpeed, strlen(udpcommands.setPlannerMaxSpeed)) == 0)
	{
		/* Проверка запущенного процесса работы станка */
		if(_runFlag == true)
		{
			sprintf(data, "%ld - STM32: Error changing! The machine is running!\n", counter);

			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}

		float speed = atof(strchr(rxUdpCharBuf, ' ') + 1);
		setPlannerMaxSpeed(&planner, speed);

		sprintf(data, "%ld - STM32: Planner max speed = %ld (steps/sec^2);\n", counter, planner._maxSpeed);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Задать _runMode */
	if(strncmp(rxUdpCharBuf, udpcommands.setDriverRunMode, strlen(udpcommands.setDriverRunMode)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		if(rxUdpCharBuf[strlen(udpcommands.setDriverRunMode) + 3] == 'V')
		{
			sprintf(data, "%ld - STM32: Movement mode driver - %d = VELOCITY_MODE;\n", counter, axis);
			setDriverRunMode(&driver[axis], VELOCITY_MODE);
		}
		else if(rxUdpCharBuf[strlen(udpcommands.setDriverRunMode) + 3] == 'P')
		{
			sprintf(data, "%ld - STM32: Movement mode driver - %d = POSITION_MODE;\n", counter, axis);
			setDriverRunMode(&driver[axis], POSITION_MODE);
		}
		else
		{
			sprintf(data, "%ld - STM32: Error changing movement mode of driver - %d;\n", counter, axis);
		}

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Задать ускорение _accel драйвера оси axis */
	if(strncmp(rxUdpCharBuf, udpcommands.setDriverAccelerationDeg, strlen(udpcommands.setDriverAccelerationDeg)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float acceleration = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverAccelerationDeg(&driver[axis], acceleration) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Acceleration of driver %d = %d (deg/sec^2);\n", counter, axis, (int16_t)acceleration);
		}
		else sprintf(data, "%ld - STM32: Error changing acceleration of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverAccelerationMm, strlen(udpcommands.setDriverAccelerationMm)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float acceleration = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverAccelerationMm(&driver[axis], acceleration) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Acceleration of driver %d = %d (mm/sec^2);\n", counter, axis, (int16_t)acceleration);
		}
		else sprintf(data, "%ld - STM32: Error changing acceleration of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverAcceleration, strlen(udpcommands.setDriverAcceleration)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		int16_t acceleration = atol(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverAcceleration(&driver[axis], acceleration) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Acceleration of driver %d = %d (steps/sec^2);\n", counter, axis, acceleration);
		}
		else sprintf(data, "%ld - STM32: Error changing acceleration of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* ---------------------- POSITION_MODE ---------------------- */

	/* Установить максимальную скорость _maxSpeed оси axis */
	if(strncmp(rxUdpCharBuf, udpcommands.setDriverMaxSpeedDeg, strlen(udpcommands.setDriverMaxSpeedDeg)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float speed = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverMaxSpeedDeg(&driver[axis], speed) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Max speed of driver %d = %d (deg/sec);\n", counter, axis, (int16_t)speed);
		}
		else sprintf(data, "%ld - STM32: Error changing max speed of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverMaxSpeedMm, strlen(udpcommands.setDriverMaxSpeedMm)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float speed = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverMaxSpeedMm(&driver[axis], speed) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Max speed of driver %d = %d (mm/sec);\n", counter, axis, (int16_t)speed);
		}
		else sprintf(data, "%ld - STM32: Error changing max speed of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverMaxSpeed, strlen(udpcommands.setDriverMaxSpeed)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float speed = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverMaxSpeed(&driver[axis], speed) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Max speed of driver %d = %d (steps/sec);\n", counter, axis, (int16_t)speed);
		}
		else sprintf(data, "%ld - STM32: Error changing max speed of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	/* Установить целевую позицию _targetPosition оси axis */
	if(strncmp(rxUdpCharBuf, udpcommands.setDriverTargetPosDeg, strlen(udpcommands.setDriverTargetPosDeg)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float target_pos = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverTargetPosDeg(&driver[axis], target_pos) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Target position of driver %d = %ld (deg);\n", counter, axis, (int32_t)target_pos);
		}
		else sprintf(data, "%ld - STM32: Error changing target position of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverTargetPosMm, strlen(udpcommands.setDriverTargetPosMm)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		float target_pos = atof(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverTargetPosMm(&driver[axis], target_pos) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Target position of driver %d = %ld (mm);\n", counter, axis, (int32_t)target_pos);
		}
		else sprintf(data, "%ld - STM32: Error changing target position of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(strncmp(rxUdpCharBuf, udpcommands.setDriverTargetPos, strlen(udpcommands.setDriverTargetPos)) == 0)
	{
		int8_t axis = atoi(strchr(rxUdpCharBuf, ' ') + 1);
		int32_t target_pos = atol(strchr(rxUdpCharBuf, ' ') + 3);

		if(setDriverTargetPos(&driver[axis], target_pos) == DRIVER_PARAM_CHANGE_OK)
		{
			sprintf(data, "%ld - STM32: Target position of driver %d = %ld (steps);\n", counter, axis, target_pos);
		}
		else sprintf(data, "%ld - STM32: Error changing target position of driver - %d;\n", counter, axis);

		udpClientSendResponseChar(data);

		memset(rxUdpCharBuf, 0, 128);
		return;
	}

	if(rxUdpCharBuf[0] == 'S')
	{
		/* VELOCITY_MODE */
		if(rxUdpCharBuf[1] == 'T' && rxUdpCharBuf[2] == 'S') // Задать целевую скорость
		{
			int16_t speed = strtol(&rxUdpCharBuf[3], NULL, 10);
			setDriverTargetSpeedDeg(&driver[0], speed);

			sprintf(data, "%ld - STM32: Target velocity (deg/sec) = %d;\n", counter, speed);
			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}
	}

	if(rxUdpCharBuf[0] == 'G')
	{
		if(rxUdpCharBuf[1] == 'C' && rxUdpCharBuf[2] == 'P') // Получить позицию
		{
			sprintf(data, "%ld - STM32: Current position = %ld;\n", counter, (int32_t)getDriverCurrentPosDeg(&driver[0]));
			udpClientSendResponseChar(data);

			memset(rxUdpCharBuf, 0, 128);
			return;
		}
	}

	sprintf(data, "%ld - STM32: Echo - %s\n", counter, rxUdpCharBuf);
	udpClientSendResponseChar(data);

	memset(rxUdpCharBuf, 0, 128);
	return;
}

/** Функция проверки связи с пинами STEP - DIR
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

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM1)
	{
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	}
}*/

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
