/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "melody_1.h"
#include "melody_2.h"
#include "melody_3.h"
#include "melody_4.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
int timeButton = 0;
int PUSHED_BUTTON;
float BRIGHTNESS1 = 0.5;
float BRIGHTNESS2 = 0.5;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifndef __GNUC__
#error Unknown compilator
#else
#include <unistd.h>
int _write(int fd, const void *buf, size_t count) {
int res = 0;
if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
	// write data to UART
	HAL_StatusTypeDef hal_res = HAL_UART_Transmit(&huart4, (uint8_t*) buf, count, HAL_MAX_DELAY);
	res = hal_res == HAL_OK ? count : -1;
	} else {
		res = -1;
}
	return res;
}
#endif

int findPin(uint16_t GPIO_Pin){
	int res = 0;

	uint16_t pinSet = 0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 , GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
		res = 1;
		pinSet = GPIO_PIN_4;
	}
	else {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 , GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
		res = 2;
		pinSet = GPIO_PIN_5;
		}
		else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
			res = 3;
			pinSet = GPIO_PIN_6;
			}
		else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 , GPIO_PIN_RESET);
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET){
				res = 4;
				pinSet = GPIO_PIN_7;
				}
			}
		}
	}



	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 , GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7 , GPIO_PIN_SET);

	return res;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
// Входной аргумент GPIO_Pin представляет из себябитовую
// маску с номером пина, откуда пришло прерывание.
// Его значение равно одной из следующих констант:
// - GPIO_PIN_0
// - GPIO_PIN_1
// - GPIO_PIN_2
// - GPIO_PIN_3
// Определим тип события (кнопка нажата или отпущена).
// Для этого просто считаем ее состояние.+
	if(HAL_GetTick() - timeButton < 300){
		return;
	}
	else
		timeButton = HAL_GetTick();
	uint16_t currentValue = HAL_GPIO_ReadPin(GPIOC, GPIO_Pin);
	if(currentValue == GPIO_PIN_RESET)
		return;
	//Фильтруем переходной процесс
	uint16_t newValue = 0;
	int timeout = 10;
	while(timeout > 0){
		 for(int i = 0, j = 1000; i < 1000; i++)
			j = i - 1;
		timeout--;
		if(newValue != currentValue){

		}
		else
			break;
		newValue = HAL_GPIO_ReadPin(GPIOC, GPIO_Pin);
	}

	//=====================================
	int btn_state = newValue;


	int res = 0;

	int displayNum ;
	//int displayNum = findPin(GPIO_Pin);

	switch(GPIO_Pin){
		case(GPIO_PIN_0):
			res = 0;
			break;
		case(GPIO_PIN_1):
				res = 1;
				break;
		case(GPIO_PIN_2):
				res = 2;
				break;
		case(GPIO_PIN_3):
				res = 3;
				break;
		default:
			res = -1;
			break;
	}
	int row = findPin(GPIO_Pin);
	if(!row || HAL_GPIO_ReadPin(GPIOC, GPIO_Pin) == GPIO_PIN_RESET)
		return;
    displayNum = 4*res + row;
	//HAL_GPIO_WritePin(GPIOE, 0xFF00, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(GPIOE, (uint8_t)displayNum << 8, GPIO_PIN_SET);
	char msg[64];
	char info[] = "Pressed button S";
	snprintf(msg, sizeof(msg), "%i", displayNum);
	HAL_UART_Transmit(&huart4, (uint8_t*)info, strlen(info), 10);
	HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 10);
	HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 10);
	PUSHED_BUTTON = displayNum;
return;
}
void reload_tim(uint32_t tim_led_cnt_tick_freq, TIM_HandleTypeDef *tim_led,    uint32_t tim_led_channel, float bright){
	uint32_t arr_value;
	uint32_t cc_value;
	arr_value = tim_led_cnt_tick_freq / 2000 - 1;
	cc_value = (arr_value + 1) * bright;
	__HAL_TIM_SET_AUTORELOAD(tim_led, arr_value);
	__HAL_TIM_SET_COMPARE(tim_led, tim_led_channel, cc_value);
}
int play_melody(int num){
	int melody_length = 0;
	uint16_t *freq;
	uint32_t cc_value_3;
	uint32_t arr_value_3;
	TIM_HandleTypeDef *tim_music = &htim3;
	uint32_t tim_melody_channel = TIM_CHANNEL_2;
	uint32_t tim_led_cnt_tick_freq_3;
	uint16_t *durations;
	HAL_UART_Transmit(&huart4, "Playing\n", 8, HAL_MAX_DELAY);

	tim_led_cnt_tick_freq_3 = SystemCoreClock;
	switch (__HAL_TIM_GET_CLOCKDIVISION(tim_music)) {
		case TIM_CLOCKDIVISION_DIV1:
			tim_led_cnt_tick_freq_3 /= 1;
		   	break;
		case TIM_CLOCKDIVISION_DIV2:
			tim_led_cnt_tick_freq_3 /= 2;
			break;
		case TIM_CLOCKDIVISION_DIV4:
			tim_led_cnt_tick_freq_3 /= 4;
			break;
	   }
	tim_led_cnt_tick_freq_3 /= (tim_music->Instance->PSC + 1);
	switch(num){
	case 1:
		freq = MELODY_1_FREQUENCIES;
		durations = MELODY_1_DURATIONS;
		melody_length = MELODY_1_LEN;
		break;
	case 2:
		freq = MELODY_2_FREQUENCIES;
		durations = MELODY_2_DURATIONS;
		melody_length = MELODY_2_LEN;
		break;
	case 3:
		freq = MELODY_3_FREQUENCIES;
		durations = MELODY_3_DURATIONS;
		melody_length = MELODY_3_LEN;
		break;
	case 4:
		freq = MELODY_4_FREQUENCIES;
		durations = MELODY_4_DURATIONS;
		melody_length = MELODY_4_LEN;
		break;
	default :
		return 1;
	}
	PUSHED_BUTTON = -1;
	for(int i = 0; i < melody_length;i++){
		if(PUSHED_BUTTON > 0)
			return 1;
	    arr_value_3 = tim_led_cnt_tick_freq_3 / *(freq + i) - 1;
	    cc_value_3 = (arr_value_3 + 1) * 0.5;
	    __HAL_TIM_SET_AUTORELOAD(tim_music, arr_value_3);
	    __HAL_TIM_SET_COMPARE(tim_music, tim_melody_channel, cc_value_3);
	    HAL_Delay(*(durations + i));
	}
	return 0;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // local variable with pointer to timer struct
   TIM_HandleTypeDef *tim_led = &htim1;
   TIM_HandleTypeDef *tim_music = &htim3;
   // timer channel
   uint32_t tim_led_channel = TIM_CHANNEL_1;
   uint32_t tim_led2_channel = TIM_CHANNEL_3;
   uint32_t tim_melody_channel = TIM_CHANNEL_2;
   // timer tick frequency CNT
   uint32_t tim_led_cnt_tick_freq;
   uint32_t tim_led_cnt_tick_freq_2;
   uint32_t tim_led_cnt_tick_freq_3;
   // helper variables for timer parameters calculations
   uint32_t arr_value;
   uint32_t cc_value;
   uint32_t arr_value_2;
   uint32_t cc_value_2;
   uint32_t arr_value_3;
   uint32_t cc_value_3;
   // reset CCR register
   __HAL_TIM_SET_COMPARE(tim_led, tim_led_channel, 0);
   // run timer
   HAL_TIM_PWM_Start(tim_led, tim_led_channel);
   HAL_TIM_PWM_Start(tim_led, tim_led2_channel);

   // calculate timer tick frequency: f_tick = f_sysclcock / (CDK * (PSK + 1))
   tim_led_cnt_tick_freq = SystemCoreClock;
   switch (__HAL_TIM_GET_CLOCKDIVISION(tim_led)) {
	   case TIM_CLOCKDIVISION_DIV1:
		   tim_led_cnt_tick_freq /= 1;
	   	   break;
	   case TIM_CLOCKDIVISION_DIV2:
		   tim_led_cnt_tick_freq /= 2;
		   break;
	   case TIM_CLOCKDIVISION_DIV4:
		   tim_led_cnt_tick_freq /= 4;
		   break;
   }
   tim_led_cnt_tick_freq /= (tim_led->Instance->PSC + 1);

   tim_led_cnt_tick_freq_2 = SystemCoreClock;
   switch (__HAL_TIM_GET_CLOCKDIVISION(tim_music)) {
	   case TIM_CLOCKDIVISION_DIV1:
		   tim_led_cnt_tick_freq_2 /= 1;
	   	   break;
	   case TIM_CLOCKDIVISION_DIV2:
		   tim_led_cnt_tick_freq_2 /= 2;
		   break;
	   case TIM_CLOCKDIVISION_DIV4:
		   tim_led_cnt_tick_freq_2 /= 4;
		   break;
   }
   tim_led_cnt_tick_freq_2 /= (tim_music->Instance->PSC + 1);
   // PWM signal
    // frequency: 2000 Hz
    // duty cycle: 0.2
    arr_value = tim_led_cnt_tick_freq / 2000 - 1;
    cc_value = (arr_value + 1) * BRIGHTNESS1;
    __HAL_TIM_SET_AUTORELOAD(tim_led, arr_value);
    __HAL_TIM_SET_COMPARE(tim_led, tim_led_channel, cc_value);

    cc_value_2 = (arr_value + 1) * BRIGHTNESS2;
    __HAL_TIM_SET_AUTORELOAD(tim_led, arr_value);
    __HAL_TIM_SET_COMPARE(tim_led, tim_led2_channel, cc_value_2);

    arr_value_3 = tim_led_cnt_tick_freq_3 / 1 - 1;
    cc_value_3 = (arr_value + 1) * 0.5;
    __HAL_TIM_SET_AUTORELOAD(tim_music, arr_value_3);
    __HAL_TIM_SET_COMPARE(tim_music, tim_melody_channel, cc_value_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(PUSHED_BUTTON){
	  case 1:
		  if(BRIGHTNESS1 < 0.89){
			  BRIGHTNESS1 += 0.1;
			  reload_tim(tim_led_cnt_tick_freq, tim_led, tim_led_channel, BRIGHTNESS1);
		  }
		  break;
	  case 2:
		  if(BRIGHTNESS1 > 0.11){
			  BRIGHTNESS1 = BRIGHTNESS1 - 0.1;
			  reload_tim(tim_led_cnt_tick_freq, tim_led, tim_led_channel, BRIGHTNESS2);
		  }
		  break;
	  case 3:
		  if(BRIGHTNESS2 < 0.89){
			  BRIGHTNESS2 += 0.1;
			  reload_tim(tim_led_cnt_tick_freq_2, tim_led, tim_led2_channel, BRIGHTNESS2);
		  }
		  break;
	  case 4:
		  if(BRIGHTNESS2 > 0.21){
			  BRIGHTNESS2 = BRIGHTNESS1 - 0.1;
			  reload_tim(tim_led_cnt_tick_freq_2, tim_led, tim_led2_channel, BRIGHTNESS2);
		  }
		  break;
	  case 5:
		  if(play_melody(1))
			  continue;
		  break;
	  case 6:
		  if(play_melody(2))
			  continue;
		  break;
	  case 7:
		  if(play_melody(3))
			  continue;
		  break;
	  case 8:
		  if(play_melody(4))
			  continue;
		  break;
	  default:
		  break;
	  }
	  PUSHED_BUTTON = -1;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 800;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 5);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
