
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#define ADC_ADDRESS (0x68 << 1)
#define INA_ADDRESS (0x41 << 1)
#define ADC_REG 0x00 // rejestr ustawien
// CTRL_REG1_A = [-RDY][C1][C0][-O/C][S1][S0][G1][G0]
#define ADC_RDY 0x80 // 1000 0000
#define ADC_O 	0x00 // 0000 0000
#define ADC_C 	0x10 // 0001 0000
#define ADC_S	0x08 // 0000 1000
#define ADC_G	0x00 // 0000 0000
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t tim=0;
uint8_t PC_message=0;
uint8_t USB_rec[8];
uint8_t USB_send[8];
uint8_t I2C_buffer[7];
static uint8_t duty=0;	// calosc okresu 0 V (odciecie tranzystora)
static uint8_t duty_lim=100;
uint8_t res=8;		// wybor rezystora pomiarowego
uint32_t res_val[8];
uint8_t Start=0;	// 1 - start pomiarow; 0 - stop pomiarow
uint8_t mod=0;		// 1 - pomiary z modulacja temperatury; 0 - brak modulacji temperatury
//uint8_t config=0;	// czy jest nowa konfiguracja do wczytania?
uint32_t ADC_wynik=0;
uint16_t USB_mes_len;
uint16_t INA_pomiar=0;
uint16_t power_lim=0;
uint8_t ADC_settings = ADC_RDY|ADC_C|ADC_S|ADC_G;
uint16_t INA_settings = 0x74;	//0x50 - shunt voltage ; 0x70 - load power
float temp=0;	//zmienna do obliczen zmiennoprzecinkowych (np. konwersja danej z ADC do wartoœci rzeczywistej)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void set_power_percentage(uint8_t percentage);
void set_MP_res(uint8_t res);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6) // Je¿eli przerwanie pochodzi od timera 6
	{
		if (duty>=duty_lim)
		{
			tim=1;
		}
		else if (duty==0)
		{
			tim=0;
		}
		if (tim==0)
		{
			duty++;
		}
		else
		{
			duty--;
		}
		set_power_percentage(duty);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  res_val[0]=24000;
  res_val[1]=51000;
  res_val[2]=75000;
  res_val[3]=100000;
  res_val[4]=240000;
  res_val[5]=510000;
  res_val[6]=1000000;
  res_val[7]=2490000;
  HAL_GPIO_WritePin(DP_PULL_GPIO_Port, DP_PULL_Pin, SET);
  HAL_GPIO_WritePin(MP_EN_GPIO_Port, MP_EN_Pin, SET);
  set_MP_res(8);
  HAL_I2C_Mem_Write(&hi2c1, INA_ADDRESS, ADC_REG, 2, &INA_settings, 2, 100);
  HAL_Delay(100);
  HAL_I2C_Master_Transmit(&hi2c1, ADC_ADDRESS, &ADC_settings, 1, 100);
  set_power_percentage(0);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc1);
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (PC_message==1)
	  {
		  if (USB_rec[0]==170)
		  {
			  USB_send[0]=85;
			  USB_mes_len=1;
			  CDC_Transmit_FS(&USB_send, USB_mes_len);
		  }
		  if (USB_rec[0]==240)
		  {
			  HAL_TIM_Base_Stop_IT(&htim6);
			  duty=0;
			  set_power_percentage(duty);
			  power_lim=256*USB_rec[1]+USB_rec[2];
			  if ((USB_rec[3]&1)==1)
				  Start=1;
			  else
			  {
				  Start=0;
				  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, RESET);
				  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, RESET);
			  }
			  if ((USB_rec[3]&2)==2)
				  mod=1;
			  else
				  mod=0;

			  res=0;
			  if ((USB_rec[3]&32)==32)
			  {
			  	  HAL_GPIO_WritePin(MP_A0_GPIO_Port, MP_A0_Pin, SET);
			  	  //HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
			  	  res=res+1;
			  }
			  else
				  HAL_GPIO_WritePin(MP_A0_GPIO_Port, MP_A0_Pin, RESET);
			  if ((USB_rec[3]&64)==64)
			  {
			  	  HAL_GPIO_WritePin(MP_A1_GPIO_Port, MP_A1_Pin, SET);
			  	  res=res+2;
			  }
			  else
				  HAL_GPIO_WritePin(MP_A1_GPIO_Port, MP_A1_Pin, RESET);
			  if ((USB_rec[3]&128)==128)
			  {
				  HAL_GPIO_WritePin(MP_A2_GPIO_Port, MP_A2_Pin, SET);
				  res=res+4;
			  }
			  else
				  HAL_GPIO_WritePin(MP_A2_GPIO_Port, MP_A2_Pin, RESET);
			  //USB_mes_len=2;
			  //CDC_Transmit_FS(&power_lim, USB_mes_len);
		  }
		  PC_message=0;
	  }

	  if (Start==1)
	  {
		  set_power_percentage(100);
		  for (int i=0; i<5; i++)
		  {
			  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			  {
				  INA_pomiar = HAL_ADC_GetValue(&hadc1);
				  temp = (INA_pomiar*3.3)/4095;
				  temp = (temp/(17.067*0.1))*1000;
				  INA_pomiar = (uint16_t)temp;
				  if (temp!=0)
				  {
					  temp=(power_lim/temp)*100;
				  }
				  if (temp>100)
				  {
					  temp=100;
				  }

				  Start=3;
				  //temp = INA_pomiar*duty/100;
				  //INA_pomiar = (uint16_t)temp;
				  //CDC_Transmit_FS(&INA_pomiar, USB_mes_len);
			  }
		  }
		  duty=(uint8_t)temp;
		  if (mod==0)
		  {
			  set_power_percentage(duty);
		  }
		  if (mod==1)
		  {
			  duty_lim=duty;
			  HAL_TIM_Base_Start_IT(&htim6);
		  }
	  }

	  if (Start==3)
	  {
		  HAL_I2C_Master_Receive(&hi2c1, ADC_ADDRESS, &I2C_buffer, 3, 100);
		  if ((I2C_buffer[2]&128)==0)
		  {
			  ADC_wynik=(I2C_buffer[0]*256+I2C_buffer[1])*625/10;
			  //ADC_wynik=(5*res_val[res]-ADC_wynik*res_val[res])/ADC_wynik;
			  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
			  {
				  INA_pomiar = HAL_ADC_GetValue(&hadc1);
				  temp = (INA_pomiar*3.3)/4095;
				  temp = (temp/(17.067*0.1))*1000;
				  INA_pomiar = (uint16_t)temp;
				  temp = INA_pomiar*duty/100;
				  INA_pomiar = (uint16_t)temp;
			  }
			  USB_send[0]=(ADC_wynik&0xFF000000)>>24;
			  USB_send[1]=(ADC_wynik&0x00FF0000)>>16;
			  USB_send[2]=(ADC_wynik&0x0000FF00)>>8;
			  USB_send[3]=ADC_wynik&0x000000FF;
			  USB_send[4]=(INA_pomiar&0xFF00)>>8;
			  USB_send[5]=INA_pomiar&0x00FF;
			  USB_send[6]=res;
			  USB_mes_len=7;
			  CDC_Transmit_FS(&USB_send, USB_mes_len);
		  }
	  }

	  /*
	  HAL_I2C_Master_Receive(&hi2c1, ADC_ADDRESS, &I2C_buffer, 4, 100);
	  ADC_wynik=(I2C_buffer[0]*256+I2C_buffer[1])*625;
	  HAL_I2C_Mem_Read(&hi2c1, INA_ADDRESS, ADC_REG, 2, &I2C_buffer, 2, 100);

	  HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	  HAL_Delay(500);
	  if (PC_message == 1)
	  {
		  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
		  {
			  INA_pomiar = HAL_ADC_GetValue(&hadc1);
			  temp = (INA_pomiar*3.3)/4095;
			  temp = (temp/(17.067*0.1))*1000;
			  INA_pomiar = (uint16_t)temp;
			  CDC_Transmit_FS(&INA_pomiar, USB_mes_len);
		  }
		  PC_message=0;
	  }*/

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1199;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MP_A0_Pin|MP_A1_Pin|MP_A2_Pin|MP_EN_Pin 
                          |LED_1_Pin|LED_2_Pin|DP_PULL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MP_A0_Pin MP_A1_Pin MP_A2_Pin MP_EN_Pin 
                           LED_1_Pin LED_2_Pin DP_PULL_Pin */
  GPIO_InitStruct.Pin = MP_A0_Pin|MP_A1_Pin|MP_A2_Pin|MP_EN_Pin 
                          |LED_1_Pin|LED_2_Pin|DP_PULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void set_power_percentage(uint8_t percentage)
{
	if (percentage>100)
		percentage=100;
	percentage=100-percentage;
	TIM2->CCR1 = percentage;

}

void set_MP_res(uint8_t res)
{
	res=res-1;
	if ((res&1)==1)
		HAL_GPIO_WritePin(MP_A0_GPIO_Port, MP_A0_Pin, SET);
	else
		HAL_GPIO_WritePin(MP_A0_GPIO_Port, MP_A0_Pin, RESET);
	if ((res&2)==2)
		HAL_GPIO_WritePin(MP_A1_GPIO_Port, MP_A1_Pin, SET);
	else
		HAL_GPIO_WritePin(MP_A1_GPIO_Port, MP_A1_Pin, RESET);
	if ((res&4)==4)
		HAL_GPIO_WritePin(MP_A2_GPIO_Port, MP_A2_Pin, SET);
	else
		HAL_GPIO_WritePin(MP_A2_GPIO_Port, MP_A2_Pin, RESET);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
