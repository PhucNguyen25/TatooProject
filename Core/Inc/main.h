/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Btn1_Pin GPIO_PIN_3
#define Btn1_GPIO_Port GPIOA
#define Btn1_EXTI_IRQn EXTI2_3_IRQn
#define Btn2_Pin GPIO_PIN_4
#define Btn2_GPIO_Port GPIOA
#define Btn2_EXTI_IRQn EXTI4_15_IRQn
#define Btn3_Pin GPIO_PIN_5
#define Btn3_GPIO_Port GPIOA
#define Btn3_EXTI_IRQn EXTI4_15_IRQn
#define Led_Pin GPIO_PIN_7
#define Led_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
	typedef struct {
		uint8_t u8Hour;
		uint8_t u8Minute;
		uint8_t u8Second;
	}Time;

  typedef enum {
    BATTERY_LEVEL_0 = 0,
    BATTERY_LEVEL_25 = 25,
    BATTERY_LEVEL_50 = 50,
    BATTERY_LEVEL_75 = 75,
    BATTERY_LEVEL_100 = 100,
  } BatteryLevel_t;

  typedef enum {
    MOTOR_SPEED_LEVEL_0 = 0,
    MOTOR_SPEED_LEVEL_25 = 25,
    MOTOR_SPEED_LEVEL_50 = 50,
    MOTOR_SPEED_LEVEL_75 = 75,
    MOTOR_SPEED_LEVEL_100 = 100,
  } MotorSpeedLevel_t;

  void            systemInit(void);
	uint8_t         ssd1306_PrintBattery(BatteryLevel_t eBatteryLevel);
	void            ssd1306_PrintTime(Time* sTime);
  void            adjustTime(Time* sTime);
  BatteryLevel_t  calculateBatteryLevel(uint16_t u16AdcVal);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
