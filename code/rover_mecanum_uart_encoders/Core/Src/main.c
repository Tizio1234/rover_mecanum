/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <mecanum.h>
#include <lwpkt_lwrb_uart.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cJSON.h>
#include <cJSON_Utils.h>

#include <queue.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// motors timers
#define FL_MOTOR_TIM htim1
#define FR_MOTOR_TIM htim1
#define BL_MOTOR_TIM htim1
#define BR_MOTOR_TIM htim1

// motors timers channels
#define FL_MOTOR_TIM_CHAN TIM_CHANNEL_1
#define FR_MOTOR_TIM_CHAN TIM_CHANNEL_2
#define BL_MOTOR_TIM_CHAN TIM_CHANNEL_3
#define BR_MOTOR_TIM_CHAN TIM_CHANNEL_4

// motors gpio config
#define FL_MOTOR_A_GPIO_PORT 		FL_MOTOR_A_GPIO_Port
#define FL_MOTOR_A_GPIO_PIN 		FL_MOTOR_A_Pin
#define FL_MOTOR_B_GPIO_PORT 		FL_MOTOR_B_GPIO_Port
#define FL_MOTOR_B_GPIO_PIN 		FL_MOTOR_B_Pin

#define FR_MOTOR_A_GPIO_PORT 		FR_MOTOR_A_GPIO_Port
#define FR_MOTOR_A_GPIO_PIN 		FR_MOTOR_A_Pin
#define FR_MOTOR_B_GPIO_PORT 		FR_MOTOR_B_GPIO_Port
#define FR_MOTOR_B_GPIO_PIN 		FR_MOTOR_B_Pin

#define BL_MOTOR_A_GPIO_PORT 		BL_MOTOR_A_GPIO_Port
#define BL_MOTOR_A_GPIO_PIN 		BL_MOTOR_A_Pin
#define BL_MOTOR_B_GPIO_PORT 		BL_MOTOR_B_GPIO_Port
#define BL_MOTOR_B_GPIO_PIN 		BL_MOTOR_B_Pin

#define BR_MOTOR_A_GPIO_PORT 		BR_MOTOR_A_GPIO_Port
#define BR_MOTOR_A_GPIO_PIN 		BR_MOTOR_A_Pin
#define BR_MOTOR_B_GPIO_PORT 		BR_MOTOR_B_GPIO_Port
#define BR_MOTOR_B_GPIO_PIN 		BR_MOTOR_B_Pin

#define lwpkt_init_event_flag 			((uint32_t)0x00000001)

#define CJSON_NUMBER_OR_ZERO(x) ((cJSON_IsNumber(x) ? cJSON_GetNumberValue(x) : 0.0f))

#define ENCODERS_MEASURE_HTIM htim10

#define FL_ENCODER_HTIM htim2
#define FR_ENCODER_HTIM htim3
#define BL_ENCODER_HTIM htim4
#define BR_ENCODER_HTIM htim5

#define ROVER_HUART huart6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* Definitions for timer */
osTimerId_t timerHandle;
const osTimerAttr_t timer_attributes = {
  .name = "timer"
};

osMessageQueueId_t mecanum_encoder_speeds_queueHandle;
const osMessageQueueAttr_t mecanum_encoder_speeds_queue_attributes = {
		.name = "mecanum_encoder_speeds_queue",
};

osSemaphoreId_t mecanum_semHandle;
const osSemaphoreAttr_t mecanum_sem_attributes = {
  .name = "mecanum_sem"
};

motor_t	fl_motor = {
		.timer = 							&FL_MOTOR_TIM,
		.channel = 						FL_MOTOR_TIM_CHAN,
		.dir_pin_1_port = 		FL_MOTOR_A_GPIO_PORT,
		.dir_pin_1 = 					FL_MOTOR_A_GPIO_PIN,
		.dir_pin_2_port = 		FL_MOTOR_B_GPIO_PORT,
		.dir_pin_2 = 					FL_MOTOR_B_GPIO_PIN
};

motor_t	fr_motor = {
		.timer = 							&FR_MOTOR_TIM,
		.channel = 						FR_MOTOR_TIM_CHAN,
		.dir_pin_1_port = 		FR_MOTOR_A_GPIO_PORT,
		.dir_pin_1 = 					FR_MOTOR_A_GPIO_PIN,
		.dir_pin_2_port = 		FR_MOTOR_B_GPIO_PORT,
		.dir_pin_2 = 					FR_MOTOR_B_GPIO_PIN
};

motor_t	bl_motor = {
		.timer = 							&BL_MOTOR_TIM,
		.channel = 						BL_MOTOR_TIM_CHAN,
		.dir_pin_1_port = 		BL_MOTOR_A_GPIO_PORT,
		.dir_pin_1 = 					BL_MOTOR_A_GPIO_PIN,
		.dir_pin_2_port = 		BL_MOTOR_B_GPIO_PORT,
		.dir_pin_2 = 					BL_MOTOR_B_GPIO_PIN
};

motor_t	br_motor = {
		.timer = 							&BR_MOTOR_TIM,
		.channel = 						BR_MOTOR_TIM_CHAN,
		.dir_pin_1_port = 		BR_MOTOR_A_GPIO_PORT,
		.dir_pin_1 = 					BR_MOTOR_A_GPIO_PIN,
		.dir_pin_2_port = 		BR_MOTOR_B_GPIO_PORT,
		.dir_pin_2 = 					BR_MOTOR_B_GPIO_PIN
};

encoder_t fl_encoder = {
		.timer = &FL_ENCODER_HTIM,
};

encoder_t fr_encoder = {
		.timer = &FR_ENCODER_HTIM,
};

encoder_t bl_encoder = {
		.timer = &BL_ENCODER_HTIM,
};

encoder_t br_encoder = {
		.timer = &BR_ENCODER_HTIM,
};

four_wheeled_robot_t robot = {
		.fl_motor = &fl_motor,
		.fr_motor = &fr_motor,
		.bl_motor = &bl_motor,
		.br_motor = &br_motor,

		.fl_encoder = &fl_encoder,
		.fr_encoder = &fr_encoder,
		.bl_encoder = &bl_encoder,
		.br_encoder = &br_encoder,
};

const cJSON_Hooks cjson_hooks = {
		.malloc_fn = pvPortMalloc,
		.free_fn = vPortFree
};

static void uart_lwpkt_evt_fn(lwpkt_t* pkt, lwpkt_evt_type_t type);

const lwpkt_lwrb_uart_init_data_t init_data = {
		.huart_p = &ROVER_HUART,
		.lwpkt_evt_fn = uart_lwpkt_evt_fn
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len);
void callback(void* argument);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  mecanum_semHandle = osSemaphoreNew(1, 1, &mecanum_sem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* Create the timer(s) */
  /* creation of timer */
  timerHandle = osTimerNew(callback, osTimerOnce, NULL, &timer_attributes);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  mecanum_encoder_speeds_queueHandle = osMessageQueueNew(1, sizeof(four_wheeled_robot_encoders_speeds_t), &mecanum_encoder_speeds_queue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8400;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 8400 - 1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000 - 1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FL_MOTOR_A_Pin|BR_MOTOR_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BL_MOTOR_A_Pin|BL_MOTOR_B_Pin|BR_MOTOR_A_Pin|FL_MOTOR_B_Pin
                          |FR_MOTOR_A_Pin|FR_MOTOR_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FL_MOTOR_A_Pin BR_MOTOR_B_Pin */
  GPIO_InitStruct.Pin = FL_MOTOR_A_Pin|BR_MOTOR_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BL_MOTOR_A_Pin BL_MOTOR_B_Pin BR_MOTOR_A_Pin FL_MOTOR_B_Pin
                           FR_MOTOR_A_Pin FR_MOTOR_B_Pin */
  GPIO_InitStruct.Pin = BL_MOTOR_A_Pin|BL_MOTOR_B_Pin|BR_MOTOR_A_Pin|FL_MOTOR_B_Pin
                          |FR_MOTOR_A_Pin|FR_MOTOR_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

static void uart_lwpkt_evt_fn(lwpkt_t* pkt, lwpkt_evt_type_t type){
	switch (type) {
		case LWPKT_EVT_PKT:
			size_t len = lwpkt_get_data_len(pkt);
			char* data = (char*)lwpkt_get_data(pkt);
			printf("Packet received, size(%d)", len);//, data(%.*s)\r\n", len, len, data);

			cJSON* parsed_json = cJSON_ParseWithLength(data, len);
			if (cJSON_IsObject(parsed_json)){
				printf("A json object\r\n");

				cJSON* power_json = cJSON_GetObjectItem(parsed_json, "p");
				cJSON* theta_json = cJSON_GetObjectItem(parsed_json, "th");
				cJSON* turn_json = cJSON_GetObjectItem(parsed_json, "tu");
				cJSON* stop_json = cJSON_GetObjectItem(parsed_json, "stop");

				osSemaphoreAcquire(mecanum_semHandle, osWaitForever);

				if (cJSON_IsTrue(stop_json)) {
					printf("Robot stopped\r\n");
					mecanum_robot_stop(&robot);
				} else {
					if (cJSON_IsNumber(power_json) || cJSON_IsNumber(theta_json) || cJSON_IsNumber(turn_json)){
						float power = CJSON_NUMBER_OR_ZERO(power_json);
						float theta = CJSON_NUMBER_OR_ZERO(theta_json);
						float turn = CJSON_NUMBER_OR_ZERO(turn_json);

						printf("Power: %f, Theta: %f, Turn: %f\r\n", power, theta, turn);

						mecanum_robot_move(&robot, power, theta, turn);
					} else {
						printf("All necessary key/value pairs missing\r\n");
					}
				}

				osSemaphoreRelease(mecanum_semHandle);

				osTimerStart(timerHandle, pdMS_TO_TICKS(300));

			} else {
				printf("Not a json object\r\n");
			}

			cJSON_Delete(parsed_json);
			break;
		default:
			break;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART6) {
    lwrb_uart_callback(Size);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == B1_Pin) {
  	if (osSemaphoreAcquire(mecanum_semHandle, 0) == osOK){
			GPIO_PinState state = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
			if (state == GPIO_PIN_SET) {
				mecanum_robot_stop(&robot);
			} else {
				mecanum_robot_move(&robot, 0.5f, 1.57f, 0.0f);
			}
	    osSemaphoreRelease(mecanum_semHandle);
  	}
  }
}

void callback(void* argument){
	printf("callback\r\n");
	if (osSemaphoreAcquire(mecanum_semHandle, 0) == osOK){
		mecanum_robot_stop(&robot);
		osSemaphoreRelease(mecanum_semHandle);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	printf("Start\r\n");

	cJSON_InitHooks(&cjson_hooks);
	printf("cJSON hooks set\r\n");

	mecanum_robot_init(&robot, HAL_GetTick());

	lwpkt_lwrb_uart_init(&init_data);
	printf("lwpkt initialized\r\n");

	HAL_TIM_Base_Start_IT(&htim10);

	/* Infinite loop */
	for(;;)
	{
		four_wheeled_robot_encoders_speeds_t speeds;
		osMessageQueueGet(mecanum_encoder_speeds_queueHandle, &speeds, NULL, osWaitForever);

		cJSON* root_object = cJSON_CreateObject();

		cJSON* fl_speed = cJSON_CreateNumber(speeds.fl_speed);
		cJSON* fr_speed = cJSON_CreateNumber(speeds.fl_speed);
		cJSON* bl_speed = cJSON_CreateNumber(speeds.fl_speed);
		cJSON* br_speed = cJSON_CreateNumber(speeds.fl_speed);

		cJSON_AddItemToObject(root_object, "fl_speed", fl_speed);
		cJSON_AddItemToObject(root_object, "fr_speed", fr_speed);
		cJSON_AddItemToObject(root_object, "bl_speed", bl_speed);
		cJSON_AddItemToObject(root_object, "br_speed", br_speed);

		char* output = cJSON_Print(root_object);
		size_t size = strlen(output);

		uart_lwpkt_write((void*)output, size);
		printf("fl:%f;fr:%f;bl:%f;br:%f\r\n", speeds.fl_speed, speeds.fr_speed, speeds.bl_speed, speeds.br_speed);

		cJSON_Delete(root_object);
		cjson_hooks.free_fn(output);

		osDelay(1000);
	}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  else if (htim->Instance == TIM10) {

  	mecanum_robot_encoders_callback(&robot, HAL_GetTick());

  	four_wheeled_robot_encoders_speeds_t speeds;
  	mecanum_robot_get_encoder_speeds(&robot, &speeds);

  	speeds.fl_speed *= 1000.0f;
  	speeds.fr_speed *= 1000.0f;
  	speeds.bl_speed *= 1000.0f;
  	speeds.br_speed *= 1000.0f;

  	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  	xQueueOverwriteFromISR((QueueHandle_t)mecanum_encoder_speeds_queueHandle, &speeds, &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

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
