/**
  ******************************************************************************
  * @file    BSP/Inc/main.h
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    13-November-2015
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f3_discovery.h"
//#include "stm32f3_discovery_accelerometer.h"
#include <stdio.h>
#include "stm32f3xx_hal.h"

#define ACC_I2C_ADDRESS						0x0
#define ACCELERO_I2C_ADDRESS				0x0
#define LSM303DLHC_WHO_AM_I_ADDR            0x0F  /* device identification register */
#define I2Cx_TIMEOUT_MAX					0x100//00
#define DISCOVERY_I2Cx_SDA_PIN              GPIO_PIN_12
#define DISCOVERY_I2Cx_SCL_PIN              GPIO_PIN_13
#define DISCOVERY_I2Cx_GPIO_PORT            GPIOB
#define DISCOVERY_I2Cx_AF                   GPIO_AF4_I2C2

/*
#define ACCELERO_DRDY_PIN                GPIO_PIN_2
#define ACCELERO_INT1_PIN                GPIO_PIN_4
#define ACCELERO_INT2_PIN                GPIO_PIN_5
#define ACCELERO_INT_GPIO_PORT           GPIOE
#define ACCELERO_DRDY_GPIO_PORT          GPIOE
#define ACCELERO_DRDY_EXTI_IRQn          EXTI2_TSC_IRQn
*/

#define ABS(x)         (x < 0) ? (-x) : x
void ACCELERO_MEMS_Test(void);
void GYRO_MEMS_Test(void);


/* Exported types ------------------------------------------------------------*/
typedef struct
{
  void   (*DemoFunc)(void);
  uint8_t DemoName[50]; 
  uint32_t DemoIndex;
}BSP_DemoTypedef;

/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Toggle_Leds(void);
void Error_Handler(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
