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
#include "stm32f1xx_hal.h"

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
