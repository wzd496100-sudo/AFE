/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define ALERT_AFE_Pin GPIO_PIN_5
#define ALERT_AFE_GPIO_Port GPIOA
#define PACK_ADC_Pin GPIO_PIN_6
#define PACK_ADC_GPIO_Port GPIOA
#define ALERT_GAUGE_Pin GPIO_PIN_1
#define ALERT_GAUGE_GPIO_Port GPIOB
#define SCL_GAUGE_Pin GPIO_PIN_10
#define SCL_GAUGE_GPIO_Port GPIOB
#define SDA_GAUGE_Pin GPIO_PIN_11
#define SDA_GAUGE_GPIO_Port GPIOB
#define I2C_SW_Pin GPIO_PIN_13
#define I2C_SW_GPIO_Port GPIOB
#define COM_3V3_EN_Pin GPIO_PIN_13
#define COM_3V3_EN_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOA
#define WAKE_AFE_Pin GPIO_PIN_15
#define WAKE_AFE_GPIO_Port GPIOA
#define CHARGE_PUMP_Pin GPIO_PIN_3
#define CHARGE_PUMP_GPIO_Port GPIOB
#define PMON_Pin GPIO_PIN_4
#define PMON_GPIO_Port GPIOB
#define PRE_DISCHAGGE_Pin GPIO_PIN_5
#define PRE_DISCHAGGE_GPIO_Port GPIOB
#define F_CON_Pin GPIO_PIN_6
#define F_CON_GPIO_Port GPIOB
#define WAKE_POWER_Pin GPIO_PIN_7
#define WAKE_POWER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
