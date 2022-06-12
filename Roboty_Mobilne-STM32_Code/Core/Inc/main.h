/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hc_sr04.h"
#include "pn532.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define KTIR_7_Pin GPIO_PIN_0
#define KTIR_7_GPIO_Port GPIOC
#define KTIR_0_Pin GPIO_PIN_1
#define KTIR_0_GPIO_Port GPIOC
#define KTIR_6_Pin GPIO_PIN_2
#define KTIR_6_GPIO_Port GPIOC
#define KTIR_1_Pin GPIO_PIN_3
#define KTIR_1_GPIO_Port GPIOC
#define KTIR_5_Pin GPIO_PIN_0
#define KTIR_5_GPIO_Port GPIOA
#define KTIR_2_Pin GPIO_PIN_1
#define KTIR_2_GPIO_Port GPIOA
#define USB_UART_TX_Pin GPIO_PIN_2
#define USB_UART_TX_GPIO_Port GPIOA
#define USB_UART_RX_Pin GPIO_PIN_3
#define USB_UART_RX_GPIO_Port GPIOA
#define KTIR_4_Pin GPIO_PIN_4
#define KTIR_4_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MOT_L_RPM_Pin GPIO_PIN_6
#define MOT_L_RPM_GPIO_Port GPIOA
#define MOT_R_RPM_Pin GPIO_PIN_7
#define MOT_R_RPM_GPIO_Port GPIOA
#define KTIR_3_Pin GPIO_PIN_0
#define KTIR_3_GPIO_Port GPIOB
#define NFC_SPI_SS_Pin GPIO_PIN_1
#define NFC_SPI_SS_GPIO_Port GPIOB
#define NFC_IRQ_Pin GPIO_PIN_10
#define NFC_IRQ_GPIO_Port GPIOB
#define NFC_SPI_SCK_Pin GPIO_PIN_13
#define NFC_SPI_SCK_GPIO_Port GPIOB
#define NFC_SPI_MISO_Pin GPIO_PIN_14
#define NFC_SPI_MISO_GPIO_Port GPIOB
#define NFC_SPI_MOSI_Pin GPIO_PIN_15
#define NFC_SPI_MOSI_GPIO_Port GPIOB
#define MOT_R_DIR_Pin GPIO_PIN_7
#define MOT_R_DIR_GPIO_Port GPIOC
#define MOT_L_PWM_Pin GPIO_PIN_8
#define MOT_L_PWM_GPIO_Port GPIOA
#define MOT_R_PWM_Pin GPIO_PIN_9
#define MOT_R_PWM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define HC_ECHO_Pin GPIO_PIN_15
#define HC_ECHO_GPIO_Port GPIOA
#define RPI_UART_TX_Pin GPIO_PIN_10
#define RPI_UART_TX_GPIO_Port GPIOC
#define RPI_UART_RX_Pin GPIO_PIN_11
#define RPI_UART_RX_GPIO_Port GPIOC
#define START_STOP_BUTTON_Pin GPIO_PIN_12
#define START_STOP_BUTTON_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MOT_L_DIR_Pin GPIO_PIN_6
#define MOT_L_DIR_GPIO_Port GPIOB
#define HC_TRIG_Pin GPIO_PIN_7
#define HC_TRIG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
