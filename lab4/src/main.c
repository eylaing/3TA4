/**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

ADC_HandleTypeDef    Adc_Handle;
ADC_ChannelConfTypeDef sConfig;
ADC_AnalogWDGConfTypeDef watchdog;


TIM_HandleTypeDef    Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;
uint16_t Tim4_PrescalerValue;


uint32_t ADC1ConvertedValue=0; 


volatile double  setPoint=23.5;

double measuredTemp; 



char lcd_buffer[6];    // LCD display buffer

enum state {setpoint = 0, monitor = 1, fan = 2};
enum state currState;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void ADC_Config(void);
void Watchdog_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);


//static void EXTILine14_Config(void); // configure the exti line4, for exterrnal button, WHICH BUTTON?


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	//HAL_ADC_MspInit(&Adc_Handle);
	
	
	HAL_Init();

	SystemClock_Config();   
	
	

	HAL_InitTick(0x0000); // set systick's priority to the highest.

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);


	BSP_LCD_GLASS_Init();
	
		BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB4");

	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	ADC_Config();
	Watchdog_Config();
	TIM4_Config();
	TIM4_OC_Config();
	
	currState = monitor;
	
 	
  while (1)
  {
		switch (currState){
			case monitor:
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"CHECK");
				HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
				HAL_ADC_Stop_DMA(&Adc_Handle);
				BSP_LED_Toggle(LED5);
				HAL_Delay(1000);
				BSP_LCD_GLASS_Clear();
				char val[6]="";
				measuredTemp=(ADC1ConvertedValue)/30.0; //??
				
				
				//sprintf((char*)val,"%f",measuredTemp);
				//BSP_LCD_GLASS_DisplayString((uint8_t*)val);
				
				sprintf((char*)val,"%d",ADC1ConvertedValue);
				BSP_LCD_GLASS_DisplayString((uint8_t*)val);
				BSP_LED_Toggle(LED5);
				HAL_Delay(2000);
				break;
			
			case setpoint:
				
				break;
			case fan:
		
				break;
		}
			
	}

}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */


void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						BSP_LCD_GLASS_DisplayString((uint8_t*)"SELECT");
						HAL_Delay(3000);
						break;	
			case GPIO_PIN_1:     //left button						
							
							break;
			case GPIO_PIN_2:    //right button						  to play again.
						
							break;
			case GPIO_PIN_3:    //up button							
				BSP_LCD_GLASS_DisplayString((uint8_t*)"UP");
						HAL_Delay(3000);
							break;
			
			case GPIO_PIN_5:    //down button						
					
							break;
			
			default://
						//default
						break;
	  } 
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																																//for timer4 
	//BSP_LED_On(LED5);
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM4_pwm
	
	__HAL_TIM_SET_COUNTER(htim, 0x0000);
		BSP_LED_Toggle(LED4);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
		BSP_LED_Toggle(LED4);
		if (currState==monitor)
		{
			currState=fan;
		}
}

void ADC_Config(void)
{
	

	

	Adc_Handle.Instance = ADC1;
	if (HAL_ADC_DeInit(&Adc_Handle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }
	
	Adc_Handle.Init.ClockPrescaler 				= ADC_CLOCK_ASYNC_DIV1;
	Adc_Handle.Init.Resolution            = ADC_RESOLUTION_12B;             /* 12-bit resolution for converted data */
  Adc_Handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  Adc_Handle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  Adc_Handle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  Adc_Handle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  Adc_Handle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  Adc_Handle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.DiscontinuousConvMode = ENABLE;                       /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
  Adc_Handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  Adc_Handle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  Adc_Handle.Init.DMAContinuousRequests = DISABLE;                        /* DMA circular mode selected */
  Adc_Handle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  Adc_Handle.Init.OversamplingMode      = DISABLE;
	
	/* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&Adc_Handle) != HAL_OK)
  {
    Error_Handler();
  }
	
	sConfig.Channel      = ADC_CHANNEL_6;                /* Sampled channel number */
  sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;    /* Sampling time (number of clock cycles unit) */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */ 
  sConfig.Offset = 0;               
     
	if (HAL_ADC_ConfigChannel(&Adc_Handle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_ADC_Start_DMA(&Adc_Handle, &ADC1ConvertedValue, 4);
}





void Watchdog_Config(void)
{
	watchdog.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  watchdog.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  watchdog.Channel = ADC_CHANNEL_6;
  watchdog.ITMode = ENABLE;
  watchdog.HighThreshold = 0x384;
  watchdog.LowThreshold = 0;
	
  if (HAL_ADC_AnalogWDGConfig(&Adc_Handle, &watchdog) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}




void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Set variable to report analog watchdog out of window status to main      */
  /* program.                                                                 */
	BSP_LED_Toggle(LED4);
	
	if (currState == monitor)
	{
		currState = fan;
	}
	
	
}

void  TIM4_Config(void)
{
 
	
	/* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = 10000; //WHAT TO CHANGE THIS TO?
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim4_Handle.Init.RepetitionCounter = 0;
	
	if (HAL_TIM_PWM_Init(&Tim4_Handle) != HAL_OK)
	{
		Error_Handler();
	}
}


void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=TIM_OCMODE_PWM1;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		Tim4_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
		Tim4_OCInitStructure.OCNPolarity=TIM_OCNPOLARITY_HIGH;
		Tim4_OCInitStructure.OCNPolarity=TIM_OCNIDLESTATE_RESET;
		Tim4_OCInitStructure.OCIdleState=TIM_OCIDLESTATE_RESET;
		
		Tim4_OCInitStructure.Pulse=(uint32_t)(666-1)*2; //this is for duty cycle
	
		HAL_TIM_PWM_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_PWM_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1);
	
	 	HAL_TIM_PWM_Start(&Tim4_Handle, TIM_CHANNEL_1); 				
		
}
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
