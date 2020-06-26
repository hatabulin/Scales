/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TM1638_STB_Pin GPIO_PIN_4
#define TM1638_STB_GPIO_Port GPIOA
#define TM1638_CLK_Pin GPIO_PIN_5
#define TM1638_CLK_GPIO_Port GPIOA
#define TM1638_DIO_Pin GPIO_PIN_7
#define TM1638_DIO_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define HX711_DATA_Pin GPIO_PIN_10
#define HX711_DATA_GPIO_Port GPIOB
#define HX711_SCK_Pin GPIO_PIN_11
#define HX711_SCK_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_9
#define USB_EN_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
void spi_Send(unsigned char data);
void spi_Read_Reg(unsigned char data_mas[]);
void DynamicBright(unsigned char num);

#define WEIGHT_FLAG 				1
#define INFO_FLAG					5
#define TARE1KG_FLAG				6
#define TARE2KG_FLAG				7
#define TARE_FLAG					2
#define SAVE_CFG_FLAG				8
#define BULK_ON_FLAG				10
#define BULK_OFF_FLAG				11
#define CONFIG_FLAG					12
#define SCALES_FLAG					13
#define AUTOTAR0_FLAG				14
#define AUTOTAR1_FLAG				15
#define CONTINUE_FLAG				16

//#define CNT_SHOW_WEIGHT 			100
//#define CNT_WEIGHT_TO_USB 			250
//#define CNT_VALUE_READ				3
#define CNT_AUTO_TARE 				10
#define WEIGHT_LIMIT_FOR_TARE		2
#define	NUM_CAL						50 // Numbers of measure values
#define LED_OR_BUZZER				0 // 0 - Led, 1 - Buzzer to indicate AUTO CALIBRATION WORKING

#define TM1638_DELAY				5
#define AFTER_SHOW_WEIGHT_DELAY		5
#define AFTER_TARE_DELAY			5

#define WEIGHT_STATUS_IDLE			1
#define WEIGHT_STATUS_NOT_CHANGE 	0
#define WEIGHT_STATUS_STABILIZED 	2
#define WEIGHT_STATUS_STAB_PROGRESS 3


#define EE_1kgLevel 	1
#define EE_2kgLevel 	3
#define EE_ZeroLevel 	5
#define EE_BulkMode  	7
#define EE_ScaleMode  	9
#define EE_AutoCalMode	11
#define EE_TareWeight	13
#define EE_UsConstant1	14
#define EEP_CHECK_ADDR 	15

#define CHECK_DATA 			0x887
#define MAX_WEIGHT_ARRAY 	100
#define TARE_WEIGHT 		1000.0f


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
