/* *** LAB2 - MECHTRON3TA4 - GROUP 2 ***************************************
The following code implements a user-delay testing application, as per the requirements.
TIM4 is configured as an OC timer to flash LEDs in the 'RESTART' mode.
TIM2 is configured as an OC timer to set a random wait time for the LEDs to come on - RNG is used.
TIM3 is configered to expire every ms, and in the PeriodElapsed callback the ms time is displayed on the LCD.

APPLICATION CONTROLS:
DOWN - returns user to 'RESTART' state to try the game again
SELECT - if in 'RESTART' mode, starts the game
RIGHT - stops the timer if the LEDs are on - if they're still off, user has cheated
LEFT - displays the best time

*/
/******************************************************************************
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
#include <string.h>

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
TIM_HandleTypeDef Tim2_Handle, Tim3_Handle, Tim4_Handle;
TIM_OC_InitTypeDef Tim2_OCInitStructure, Tim4_OCInitStructure;
uint16_t Tim4_PrescalerValue, Tim3_PrescalerValue;
uint32_t Tim2_PrescalerValue;
__IO uint16_t Tim4_CCR;
__IO uint32_t Tim2_CCR;
char lcd_buffer[6];    // LCD display buffer



__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};// the emulated EEPROM can save 3 varibles, at these three addresses.

uint32_t VARIABLE;
char x[6];
uint16_t num = 0;
typedef enum {RESTART, BESTTIME, LEDOFF, LEDON, PRESSED} STATES;
STATES curr_state=RESTART;

RNG_HandleTypeDef Rng_Handle;


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM2_Config(void);
void TIM2_OC_Config(void);
void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);

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
	
	HAL_Init();

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	//Initializing LEDs, LCD, Joystick needed for app
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB 2");	

//******************* use emulated EEPROM ====================================
	HAL_FLASH_Unlock();
		
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
	
	//test write to/read from emulated EEPROM
	/*
	BSP_LCD_GLASS_Clear();
	uint16_t testWrite=5000;
	uint16_t testRead;
	EE_status = EE_WriteVariable(VirtAddVarTab[0], testWrite);
	EE_status |= EE_ReadVariable(VirtAddVarTab[0], &testRead);
	//Check that write/read worked:
	if (testWrite == testRead) {BSP_LCD_GLASS_DisplayString((uint8_t*)"OKAY");}
	else if (testWrite != testRead) {BSP_LCD_GLASS_DisplayString((uint8_t*)"FAIL");}
	*/
	
//*********************use RNG ================================  
	Rng_Handle.Instance=RNG; 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
	//test of RNG	
	/*
	uint32_t testVar = (HAL_RNG_GetRandomNumber(&Rng_Handle))%5000;
	char rngBuff[10];
	sprintf(rngBuff,"%u",testVar);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)rngBuff);
	*/
	
	
	TIM3_Config();
	TIM4_Config();
	Tim4_CCR=5000;
	TIM4_OC_Config();

  /* Infinite loop */
  while (1)
  {	

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

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}

void  TIM2_Config(void)
{
  
  /* Compute the prescaler value to have TIM2 counter clock equal to 10 KHz */
  Tim2_PrescalerValue = (uint32_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM2 instance */
  Tim2_Handle.Instance = TIM2;
 
  Tim2_Handle.Init.Period = 60000;
  Tim2_Handle.Init.Prescaler = Tim2_PrescalerValue;
  Tim2_Handle.Init.ClockDivision = 0;
  Tim2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim2_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    Error_Handler();
  }
  
  /*Start the TIM Base generation in interrupt mode ####################*/
  if(HAL_TIM_Base_Start_IT(&Tim2_Handle) != HAL_OK)  
  {
    Error_Handler();
  }
	
}

void  TIM2_OC_Config(void)
{
		Tim2_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim2_OCInitStructure.Pulse=Tim2_CCR;
		Tim2_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim2_Handle); 
	
		HAL_TIM_OC_ConfigChannel(&Tim2_Handle, &Tim2_OCInitStructure, TIM_CHANNEL_1);
	
	 	HAL_TIM_OC_Start_IT(&Tim2_Handle, TIM_CHANNEL_1); 	
}


void  TIM3_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = 10-1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    Error_Handler();
  }
  
  /*Start the TIM Base generation in interrupt mode ####################*/
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)  
  {
    Error_Handler();
  }
}

void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = 40000;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
}




void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1);
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); 				
		
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*NOTE - priority for joystick interrupts is set as priority 1 in stm32l476g_discovery.c to allow
		for user actions (joystick) to override the timer interrupts*/
  switch (GPIO_Pin) {
			case GPIO_PIN_0:				//SELECT button	- game start				
				switch (curr_state) {
					case RESTART: //start the game if it's in restart mode
						BSP_LCD_GLASS_Clear();
						BSP_LED_Off(LED5);
						BSP_LED_Off(LED4);
						VARIABLE = (HAL_RNG_GetRandomNumber(&Rng_Handle))%50000;
						TIM2_Config();
						Tim2_CCR = VARIABLE;
						TIM2_OC_Config();
						curr_state=LEDOFF; //LEDs are now off
							break;
					default:
							break;
				}			
						break;	
			
			case GPIO_PIN_1:     //left button - display best time				
						curr_state=BESTTIME;
						BSP_LCD_GLASS_Clear();
						//Display best time - retrieve from EEPROM
						uint16_t time;
						EE_ReadVariable(VirtAddVarTab[0], &time);
						char timeBuffer[6];
						sprintf(timeBuffer,"%d",time); //cast int to char to display on LCD
						BSP_LCD_GLASS_DisplayString((uint8_t*)timeBuffer);				
							break;
			
			case GPIO_PIN_2:    //right button - stops timer
					switch(curr_state){
						case LEDON: //not cheating, valid
							NVIC_DisableIRQ(TIM3_IRQn);
							curr_state=PRESSED;
							uint16_t prevBest;
							EE_ReadVariable(VirtAddVarTab[0], &prevBest);
							if (num<prevBest||prevBest==0){
								//replace EEPROM value with new best or first record
								EE_WriteVariable(VirtAddVarTab[0], num);
							}
							num=0; //reset num for next game
								break;
						case LEDOFF: //cheating, LEDs not on yet
							curr_state=RESTART; //go back to flashing LEDs
							BSP_LCD_GLASS_DisplayString((uint8_t*)"CHEAT");
								break;
						default:
								break;
					}
							break;
			
			case GPIO_PIN_3:    //up button	- no function in application						
					BSP_LCD_GLASS_Clear();			
							break;
			
			case GPIO_PIN_5:    //down button	- return to restart					
					curr_state=RESTART;
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"RETRY");
							break;
			default:
						break;
	  } 
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32lxx_hal_tim.c for different callback function names. 																															//for timer 3 , Timer 3 use update event initerrupt
{
	//fires when TIM3 expires - counts ms
	if(curr_state==LEDON){ //only incrementing if the LEDs are on and waiting for user input
		sprintf(x, "%d", num); //cast to string for display
		BSP_LCD_GLASS_Clear();
		BSP_LCD_GLASS_DisplayString((uint8_t*) x);
		num++;
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	if ((*htim).Instance==TIM4 && curr_state==RESTART){ //only toggling LEDs in RESTART state
		BSP_LED_Toggle(LED4);
		BSP_LED_Toggle(LED5);
		//clear the timer counter at the end of call back to avoid interrupt interval variation
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h
	}
	
	else if((*htim).Instance==TIM2 && curr_state==LEDOFF){ //RNG timer expires, turn on LEDs
	BSP_LED_On(LED4);
	BSP_LED_On(LED5);
	curr_state=LEDON;
	NVIC_EnableIRQ(TIM3_IRQn); //start TIM3 to keep track of user delay
	TIM3_Config();
	}
}


static void Error_Handler(void)
{
 
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
