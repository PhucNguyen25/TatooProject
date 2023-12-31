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
#include "adc.h"
#include "i2c.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	uint32_t u32StartedTick;
	BatteryLevel_t eBattery;
	Time sTime;
	uint16_t u16AdcValue = 0;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	//Init User application param
	systemInit();

	//Init Oled and fill Black
	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();

	u32StartedTick = HAL_GetTick();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//For clock
		if (HAL_GetTick() - u32StartedTick > 1000)
		{
			u32StartedTick = HAL_GetTick();
			sTime.u8Second++;
			adjustTime(&sTime);
			ssd1306_PrintTime(&sTime);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}

		//For Battery
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 300);
    u16AdcValue = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);
		eBattery = calculateBatteryLevel(u16AdcValue);
		ssd1306_PrintBattery(eBattery);
		
		HAL_Delay(100);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void systemInit(void)
{
	eBattery = BATTERY_LEVEL_0;
	sTime.u8Hour = 0;
	sTime.u8Minute = 0;
	sTime.u8Second = 0;
}

uint8_t ssd1306_PrintBattery(BatteryLevel_t eBatteryLevel)
{
	if (eBatteryLevel > 100)
	{
		return 0;
	}
	//Clear battery region
	ssd1306_FillRectangle(3,3,13,61,Black);
	ssd1306_FillRectangle(40,10,127,36,Black);

  //Visable Batterry Level
  char BatteryStr[4] = {0};
	sprintf(BatteryStr, "%d%%", eBatteryLevel);
  ssd1306_SetCursor(40,10);
	ssd1306_WriteString(BatteryStr, Font_16x26, White);

  //Visable Batterry Column
	if (eBatteryLevel == BATTERY_LEVEL_0)
	{
		ssd1306_DrawRectangle(3,3,13,13,White);
		ssd1306_DrawRectangle(3,19,13,29,White);
		ssd1306_DrawRectangle(3,35,13,45,White);
		ssd1306_DrawRectangle(3,51,13,61,White);
	}
	else if (eBatteryLevel <= BATTERY_LEVEL_25)
	{
		ssd1306_DrawRectangle(3,3,13,13,White);
		ssd1306_DrawRectangle(3,19,13,29,White);
		ssd1306_DrawRectangle(3,35,13,45,White);
		ssd1306_FillRectangle(3,51,13,61,White);
	}
	else if (eBatteryLevel <= BATTERY_LEVEL_50)
	{
		ssd1306_DrawRectangle(3,3,13,13,White);
		ssd1306_DrawRectangle(3,19,13,29,White);
		ssd1306_FillRectangle(3,35,13,45,White);
		ssd1306_FillRectangle(3,51,13,61,White);
	}
	else if (eBatteryLevel <= BATTERY_LEVEL_75)
	{
		ssd1306_DrawRectangle(3,3,13,13,White);
		ssd1306_FillRectangle(3,19,13,29,White);
		ssd1306_FillRectangle(3,35,13,45,White);
		ssd1306_FillRectangle(3,51,13,61,White);
	}
	else if (eBatteryLevel <= BATTERY_LEVEL_100)
	{
		ssd1306_FillRectangle(3,3,13,13,White);
		ssd1306_FillRectangle(3,19,13,29,White);
		ssd1306_FillRectangle(3,35,13,45,White);
		ssd1306_FillRectangle(3,51,13,61,White);
	}

	ssd1306_UpdateScreen();

	return 1;
}

void adjustTime(Time* sTime) {
    if (sTime->u8Second > 59) {
        sTime->u8Second = 0;
        sTime->u8Minute++;
    }

    if (sTime->u8Minute > 59) {
        sTime->u8Minute = 0;
        sTime->u8Hour++;
    }
		
    if (sTime->u8Hour > 99) {
        sTime->u8Hour = 0;
    }
}

BatteryLevel_t calculateBatteryLevel(uint16_t u16AdcVal) {
	return (BatteryLevel_t)(u16AdcVal/40);
}


void ssd1306_PrintTime(Time* sTime)
{
	//Validate input
	if ((sTime->u8Minute > 59) || (sTime->u8Second > 59))
		return;

  char Temp[9] = "00:10:08";
	sprintf(Temp, "%02d:%02d:%02d", sTime->u8Hour, sTime->u8Minute, sTime->u8Second);
  ssd1306_SetCursor(25,45);
	ssd1306_WriteString(Temp, Font_11x18, White);
	ssd1306_UpdateScreen();
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
