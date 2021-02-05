/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "app_mems.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//String library
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_BUF_SIZE	1000
#define Teseo_I2C_7bits_Addr	0x3A
#define MAX_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart2;

/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for robotArms */
osThreadId_t robotArmsHandle;
const osThreadAttr_t robotArms_attributes = {
  .name = "robotArms",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for robotWheels */
osThreadId_t robotWheelsHandle;
const osThreadAttr_t robotWheels_attributes = {
  .name = "robotWheels",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for robotRockets */
osThreadId_t robotRocketsHandle;
const osThreadAttr_t robotRockets_attributes = {
  .name = "robotRockets",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for robotHips */
osThreadId_t robotHipsHandle;
const osThreadAttr_t robotHips_attributes = {
  .name = "robotHips",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
//i2c handler for MEMS and GNSS
I2C_HandleTypeDef hi2c1;

//Data for ADC1(X-axis) and ADC2(Y-axis)
uint32_t adcDataX[1];
uint32_t adcDataY[1];

//USB variables
ApplicationTypeDef Appli_state;
USBH_HandleTypeDef hUsbHostFS;

//GNSS variables
static const char *gpgll_msg = "$PSTMNMEAREQUEST,100000,0\n\r";
uint8_t read_buf[I2C_BUF_SIZE];
char GNSSposition[2000];

//MEMS data
extern char Acceleration[MAX_BUF_SIZE];
extern char Gyroscope[MAX_BUF_SIZE];
extern char Magnetometer[MAX_BUF_SIZE];
extern char Temperature[MAX_BUF_SIZE];
extern char Humidity[MAX_BUF_SIZE];
extern char Pressure[MAX_BUF_SIZE];

//RTC variables
RTC_TimeTypeDef sTime = { 0 };
RTC_DateTypeDef sDate = { 0 };
RTC_AlarmTypeDef sAlarm = { 0 };
char timestamp[30];

//Flags
int MEMSready;
int GNSSready;
int direction;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void writeToUsbTask(void *argument);
void moveRobotArms(void *argument);
void moveRobotWheels(void *argument);
void moveRobotRockets(void *argument);
void moveRobotHips(void *argument);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
//Write to console
void UART_Write(uint8_t *string);
//Initialize colum names
void ColumnNamesInit(void);
//Write timestamp to USB
void RTC_Process(void);
//Write MEMS data to USB
void MX_MEMS_Process(void);
//Write GNSS data to USB
void GNSS_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	UART_Write("RTC_AlarmAEventCallback\r\n");
}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_MEMS_Init();
  /* USER CODE BEGIN 2 */

//Timer for MEMS_Process(); GNSS_Process();
HAL_TIM_Base_Start_IT(&htim7);

//Start ADC for x-axis (arms) of joystick
HAL_ADC_Start_DMA(&hadc1, adcDataX, 1);

//Control speed of motor arms
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

//Start ADC for y-axis (wheels) of joystick
HAL_ADC_Start_DMA(&hadc2, adcDataY, 1);

//Control speed of right motor wheel
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//Control speed of left motor wheel
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

//Control speed of motor hips
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

//Control speed of motor rockets
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of usbTask */
  usbTaskHandle = osThreadNew(writeToUsbTask, NULL, &usbTask_attributes);

  /* creation of robotArms */
  robotArmsHandle = osThreadNew(moveRobotArms, NULL, &robotArms_attributes);

  /* creation of robotWheels */
  robotWheelsHandle = osThreadNew(moveRobotWheels, NULL, &robotWheels_attributes);

  /* creation of robotRockets */
  robotRocketsHandle = osThreadNew(moveRobotRockets, NULL, &robotRockets_attributes);

  /* creation of robotHips */
  robotHipsHandle = osThreadNew(moveRobotHips, NULL, &robotHips_attributes);

  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
	while (1) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 14;
  sTime.Minutes = 15;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 14;
  sAlarm.AlarmTime.Minutes = 15;
  sAlarm.AlarmTime.Seconds = 10;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 256-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 256-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 80-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 256-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB13
                           PB14 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UART_Write(uint8_t *string) {
	HAL_UART_Transmit(&huart2, string, strlen((char*) string), 1000);
}

void ColumnNamesInit(void) {
	USB_Write("Timestamp;ACC_X;ACC_Y;ACC_Z;GYR_X;GYR_Y;GYR_Z;");
	USB_Write("MAG_X;MAG_Y;MAG_Z;Temp;Hum;Press;;GNSS Position;\r\n");
}

void RTC_Process(void) {
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_Delay(10);
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_Delay(10);

	sprintf((char*) timestamp, "%02d:%02d:%02d    %02d.%02d.%02d	;",
			sTime.Hours, sTime.Minutes, sTime.Seconds, sDate.Date, sDate.Month,
			sDate.Year);

	USB_Write(timestamp); //write timestamp on USB
}

void GNSS_Process(void) {

	int i;
	bool GPSfound = 0;
	bool statusSent = 0;

	HAL_I2C_Master_Transmit(&hi2c1, Teseo_I2C_7bits_Addr << 1, (uint8_t*) gpgll_msg, strlen(gpgll_msg), 2000);

	//UART_Write("got:...\n\r");

	for (read_buf[I2C_BUF_SIZE - 1] = 0; read_buf[I2C_BUF_SIZE - 1] != 0xff;) {
		HAL_I2C_Master_Receive(&hi2c1, Teseo_I2C_7bits_Addr << 1, read_buf, I2C_BUF_SIZE, 2000);
		if (read_buf[0] == NULL) {
			UART_Write("Buffer empty...\n\r");
		} else {
			for (int i = 0; i < I2C_BUF_SIZE; i++) {
				if (read_buf[i] != 0xff) {
					GPSfound = 1;
					HAL_UART_Transmit(&huart2, &read_buf[i], 1, 1000);
					strncat(GNSSposition, &read_buf[i], 1);

				} else if (!GPSfound) {
					if (!statusSent) {
						UART_Write("Searching for GPS Data...\n\r");
						statusSent = 1;
					}
				}

			}

		}

	}

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_writeToUsbTask */
/**
 * @brief  Function implementing the usbTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_writeToUsbTask */
void writeToUsbTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
	bool oneTimeColumnNamesInitDone = 0;
	/* Infinite loop */
	for (;;) {

		USBH_Process(&hUsbHostFS);

		switch (Appli_state) {
		case APPLICATION_IDLE:
			break;

		case APPLICATION_START:
			break;

		case APPLICATION_READY:
			if (MEMSready == 1 && GNSSready == 1) {
				UART_Write(
						"-------------------------------------USB STARTING-------------------------------------\r\n");

				//executed once after every reset, write type of sensors on USB
				if (!oneTimeColumnNamesInitDone) {
					ColumnNamesInit();
					oneTimeColumnNamesInitDone = 1;
				}

				RTC_Process();
				UART_Write("RTC Data written to USB\r\n");

				USB_Write(Acceleration);
				USB_Write(Gyroscope);
				USB_Write(Magnetometer);
				USB_Write(Temperature);
				USB_Write(Humidity);
				USB_Write(Pressure);
				MEMSready = 0;
				UART_Write("MEMS Data written to USB, MEMSready=0\r\n");

				USB_Write(";");

				USB_Write(GNSSposition);
				GNSSready = 0;
				//clear array
				GNSSposition[0] = '\0';
				UART_Write("GNSS Data written to USB, GNSSready=0\r\n");

				UART_Write(
						"-------------------------------------USB FINISHED-------------------------------------\r\n");

			}
			break;

		case APPLICATION_DISCONNECT:
			break;
		}

		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_moveRobotArms */
/**
 * @brief Function implementing the robotArms thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_moveRobotArms */
void moveRobotArms(void *argument)
{
  /* USER CODE BEGIN moveRobotArms */
	/* Infinite loop */
	for (;;) {
		HAL_ADC_Start(&hadc1);

		if (adcDataX[0] > 120) {
			//arms apart
			TIM3->CCR1 = adcDataX[0];
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);

//			UART_Write("Arms apart\r\n");

		} else if (adcDataX[0] < 100) {

			//arms together
			TIM3->CCR1 = 255 - adcDataX[0];
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
//			UART_Write("Arms together\r\n");


		} else {
			//still
			TIM3->CCR1 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
//			UART_Write("Arms don't move\r\n");
		}

		osDelay(1);
	}
  /* USER CODE END moveRobotArms */
}

/* USER CODE BEGIN Header_moveRobotWheels */
/**
* @brief Function implementing the robotWheels thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveRobotWheels */
void moveRobotWheels(void *argument)
{
  /* USER CODE BEGIN moveRobotWheels */
  /* Infinite loop */
  for(;;)
  {
	  		HAL_ADC_Start(&hadc2);
	  //		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0 || HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) {
	  		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
	  //		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) {

	  			if (direction == 0) {
	  				direction = 1;
	  				while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
	  //				while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) {
	  					//right wheel forwards
	  					TIM4->CCR1 = 160;
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

	  					//left wheel backwards
	  					TIM4->CCR2 = 160;
	  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

//	  					UART_Write("Turn Left\r\n");
	  				}
	  			}

	  			else {
	  				direction = 0;
	  //				while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0) {
	  					while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
	  					//right wheel backwards
	  					TIM4->CCR1 = 160;
	  					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	  					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

	  					//left wheel forwards
	  					TIM4->CCR2 = 160;
	  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	  					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

//	  					UART_Write("Turn Right\r\n");
	  				}
	  			}

	  		}

	  		//ADC
	  		else {
	  //			UART_Write("else\r\n");
	  			if (adcDataY[0] > 120) {
	  				//right wheel forwards
	  				TIM4->CCR1 = adcDataY[0];
	  				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	  				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

	  				//left wheel forwards
	  				TIM4->CCR2 = adcDataY[0];
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);

//	  				UART_Write("Wheels move forwards\r\n");

	  			} else if (adcDataY[0] < 100) {

	  				//right wheel backwards
	  				TIM4->CCR1 = 255 - adcDataY[0];;
	  				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	  				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

	  				//left wheel backwards
	  				TIM4->CCR2 = 255 - adcDataY[0];;
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);

//	  				UART_Write("Wheels move backwards\r\n");

	  			} else {
	  				//right wheel doesn't move
	  				TIM4->CCR1 = 0;
	  				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
	  				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);

	  				//left wheel doesn't move
	  				TIM4->CCR2 = 0;
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
	  				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);

	  //			UART_Write("Wheels don't move\r\n");
	  			}
	  		}

    osDelay(1);
  }
  /* USER CODE END moveRobotWheels */
}

/* USER CODE BEGIN Header_moveRobotRockets */
/**
* @brief Function implementing the robotRocket thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_moveRobotRockets */
void moveRobotRockets(void *argument)
{
  /* USER CODE BEGIN moveRobotRockets */
  /* Infinite loop */
  for(;;)
  {
	  	  	//Rockets left
	  		TIM2->CCR4 = 250;
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);

	  		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {

	  			//Rocket right
	  			TIM2->CCR4 = 250;
	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
//	  			UART_Write("Rocket right\r\n");
	  		}
    osDelay(1);
  }
  /* USER CODE END moveRobotRockets */
}

/* USER CODE BEGIN Header_moveRobotHips */
/**
 * @brief Function implementing the robotHips thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_moveRobotHips */
void moveRobotHips(void *argument)
{
  /* USER CODE BEGIN moveRobotHips */
	/* Infinite loop */
	for (;;) {

		//Robot bends
		TIM2->CCR1 = 180;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);

		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0) {
			TIM2->CCR1 = 180;

			//Robot stretches
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
//			UART_Write("Robot stretches\r\n");

		}
		osDelay(1);
	}
  /* USER CODE END moveRobotHips */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
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
	if (htim->Instance == TIM7) {
		if (Appli_state == APPLICATION_READY) {
			UART_Write(
					"------------------------------------MEMS PROCESS STARTING------------------------------------\r\n");
			UART_Write("\r\n");
			MX_MEMS_Process();
			MEMSready = 1;
			UART_Write("MEMS Data aquired, MEMSready=1\r\n");
			UART_Write(
					"------------------------------------MEMS PROCESS FINISHED------------------------------------\r\n");

			UART_Write("-\r\n");

			UART_Write(
					"------------------------------------GNSS PROCESS STARTING------------------------------------\r\n");
			UART_Write("\r\n");
			GNSS_Process();
			GNSSready = 1;
			UART_Write("GNSS Data aquired, GNSSready=1\r\n");
			UART_Write(
					"------------------------------------GNSS PROCESS FINISHED------------------------------------\r\n");

			UART_Write("-\r\n-\r\n");

		} else {
			UART_Write("MEMS and GNSS Data NOT aquired, USB not ready\r\n");
		}

	}
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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
