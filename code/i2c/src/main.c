/**
 ******************************************************************************
 * @file    BSP/Src/main.c
 * @author  MCD Application Team
 * @version V1.4.0
 * @date    13-November-2015
 * @brief   This example code shows how to use the STM32F3-Discovery BSP Drivers
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup TM32F3xx_HAL_Examples
 * @{
 */

/** @addtogroup BSP_Example
 * @{
 */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Init af threahold to detect acceleration on MEMS */
int16_t ThresholdHigh = 1000;
int16_t ThresholdLow = -1000;
/* Private function prototypes -----------------------------------------------*/
//static void ACCELERO_ReadAcc(void);

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	/* STM32F3xx HAL library initialization:
	   - Configure the Flash prefetch
	   - Systick timer is configured by default as source of time base, but user 
	   can eventually implement his proper time base source (a general purpose 
	   timer for example or other time source), keeping in mind that Time base 
	   duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
	   handled in milliseconds basis.
	   - Set NVIC Group Priority to 4
	   - Low Level Initialization
	 */
	//	 setbuf(stdout, NULL);
	//	 printf("Begining of the progam\n");
	HAL_Init();
	HAL_InitTick(0);
	//HAL_NVIC_SetPriority(-1, 0 ,0);

	/* Configure the system clock to 72 Mhz */
	SystemClock_Config();

	// Initialize LEDs and User_Button on STM32F3-Discovery
	GPIO_InitTypeDef  GPIO_InitStruct;

	// Enable the GPIO_LED Clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5|GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	// i2c Init

	static I2C_HandleTypeDef I2cHandle;
	uint32_t I2cxTimeout = I2Cx_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */

	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable DRDY clock

	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
	//while(1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	if(HAL_I2C_GetState(&I2cHandle) == HAL_I2C_STATE_RESET)
	{
		I2cHandle.Instance = I2C2;
		I2cHandle.Init.OwnAddress1 =  ACCELERO_I2C_ADDRESS; // 0x32
		I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		I2cHandle.Init.OwnAddress2 = 0;
		I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;	

		// Init the I2C
		GPIO_InitTypeDef GPIO_InitStructure;

		// Enable SCK and SDA GPIO clocks
		//__HAL_RCC_GPIOB_CLK_ENABLE();

		// I2Cx SD1 & SCK pin configuration
		GPIO_InitStructure.Pin = (DISCOVERY_I2Cx_SDA_PIN | DISCOVERY_I2Cx_SCL_PIN);
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStructure.Alternate = DISCOVERY_I2Cx_AF;
		HAL_GPIO_Init(DISCOVERY_I2Cx_GPIO_PORT, &GPIO_InitStructure);

		// Enable the I2C clock
		__HAL_RCC_I2C1_CLK_ENABLE();

		HAL_I2C_Init(&I2cHandle);
	}

	// end I2C Init
	//	 printf("I2C initialization finished\n");

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	uint8_t ctrl = 0x00;
	while (1)
	{
		//  printf("Emitting\n");
		//ctrl = COMPASSACCELERO_IO_Read(ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR);

		HAL_StatusTypeDef status = HAL_OK;
		status = HAL_I2C_Mem_Read(&I2cHandle, ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR, I2C_MEMADD_SIZE_8BIT, &ctrl, 1, I2cxTimeout);
		//  printf("Reading\n");

		// Check the communication status
		if(status != HAL_OK)
		{
			//I2Cx_Error();
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		}

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
		//printf("%#08x\n",ctrl);
		//ACCELERO_ReadAcc();
		//	printf("B on\n");
		//	printf("Delay start\n");
		//HAL_Delay(1000);
		int i,a = 0;
		for(i=0;i<100;++i)
			++a;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);

		//	printf("Delay end\n");
		//	printf("B off\n");
	}
/*
*/
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV                     = 1
 *            PLLMUL                         = 9
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;


	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	while(1)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	}
}
