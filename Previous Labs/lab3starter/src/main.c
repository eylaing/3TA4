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


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

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
I2C_HandleTypeDef  pI2c_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, or HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  //   
char datestring[6]={0};
uint8_t timedate[50]={0};
uint8_t SP=1; //since 0 is used to store SP in EEPROM
uint8_t dsmode = 0;
uint8_t state = 0;
enum Setting {YEAR=0, MONTH=1, WDAY=2, DAY=3, HOUR=4, MIN=5, SEC=6};

uint8_t wd, dd, mo, yy, ss, mm, hh; // for weekday, day, month, year, second, minute, hour

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed, pbpressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void RTC_Config(void);
void RTC_AlarmAConfig(void);
void RTC_TimeShow(void);
void Display_Curr_Setting(enum Setting SET);
void RTC_DateDisplay(void);
void RTC_SavePressTime(void);
void EEPROM_SaveTime(uint8_t hour, uint8_t min, uint8_t sec);
void EEPROM_ReadTime(uint8_t SPoffset);
void Pushbutton_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	leftpressed=0;
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;
	enum Setting dtSetting = -1;
	
	HAL_Init();
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	Pushbutton_Init(); //initialize the external pushbutton

	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	HAL_Delay(1000);

	//configure real-time clock
	RTC_Config();
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);

	SP=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);
	HAL_Delay(10);

/**********************Testing I2C EEPROM------------------
	uint8_t data1 =0x67,  data2=0x68;
	uint8_t readData=0x00;
	uint16_t EE_status;

	//TEST 1 - Write data1
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");
	HAL_Delay(1000);
	
	//TEST 2 - Write data2
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");
	HAL_Delay(1000);
	
	//TEST 3 - Read data1
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);
	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	HAL_Delay(1000);
	
	//TEST 4 - Read data2
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);
	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	
	HAL_Delay(1000);		
*/
	while (1) 
  {
		if(dsmode!=1) //Not in dateset mode
		{			
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>1000) {	//held for 1s						
							RTC_AlarmA_IT_Disable(&RTCHandle); //disable RTC to prevent time being displayed
							RTC_DateDisplay();
						} 
					}
					RTC_AlarmA_IT_Enable(&RTCHandle);
			}		
			
			if (selpressed==1)  { 
					selpressed=0;
			} 
			
			if (uppressed==1) {
					uppressed=0;
			}
			if (pbpressed==1) { //Save to EEPROM
					RTC_SavePressTime();
					RTC_AlarmA_IT_Enable(&RTCHandle);
					pbpressed=0;
			}
			if (leftpressed==1) { //Display last 2
				RTC_AlarmA_IT_Disable(&RTCHandle);
				if (SP<6) { //there aren't 2 times saved
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"2FEW");
					HAL_Delay(1000);
				}
				else {
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"READ2");
					HAL_Delay(1000);
					EEPROM_ReadTime(0);
					//HAL_Delay(2000); //optional to display time longer
					EEPROM_ReadTime(3);
					//HAL_Delay(3000);
				}
				RTC_AlarmA_IT_Enable(&RTCHandle);
					leftpressed=0;
			}			
			
			if (rightpressed==1) { //Switch into Datetime set mode
				BSP_LCD_GLASS_DisplayString((uint8_t*) "DSMODE");
				dsmode=1;
				rightpressed=0;
			}
			if (downpressed==1){downpressed=0;}
		}

		else if(dsmode==1) //In dateset mode
		{
			if (uppressed==1) //up was pressed - increment current setting
			{
					switch(dtSetting) {
					case YEAR: 
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Year++;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Year);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);					
						break;
					}
					case MONTH:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Month++;
						if(RTC_DateStructure.Month>12) RTC_DateStructure.Month = 1; //only can have 12 months
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Month);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
					}
					case WDAY:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.WeekDay++;
						if(RTC_DateStructure.WeekDay>7) RTC_DateStructure.WeekDay = 1; //only 7 weekdays
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.WeekDay);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
					}
					case DAY:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Date++;
						if(RTC_DateStructure.Date>31) RTC_DateStructure.Date = 1; //max 31 days in a month
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Date);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
					}
					case HOUR:
					{							
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Hours++;
						if(RTC_TimeStructure.Hours==24) RTC_TimeStructure.Hours = 0; //only 0-23 hours on 24hr clock
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Hours);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}	
					case MIN:
					{
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Minutes++;
						if(RTC_TimeStructure.Minutes>=60) RTC_TimeStructure.Minutes = 0; 
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Minutes);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}
					case SEC:
					{
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Seconds++;
						if(RTC_TimeStructure.Seconds>=60) RTC_TimeStructure.Seconds = 0;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Seconds);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}
				}
				uppressed=0;
			}
		
			if (downpressed==1) //down was pressed - decrement current setting
			{
					switch(dtSetting) {
					case YEAR: 
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Year--;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Year);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);					
						break;
					}
					case MONTH:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Month--;
						if(RTC_DateStructure.Month<1) RTC_DateStructure.Month = 12;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Month);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
					}
					case WDAY:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.WeekDay--;
						if(RTC_DateStructure.WeekDay<1) RTC_DateStructure.WeekDay = 7;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.WeekDay);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
						break;
					}
					case DAY:
					{
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_DateStructure.Date--;
						if(RTC_DateStructure.Date<1) RTC_DateStructure.Date = 31;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d", RTC_DateStructure.Date);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						break;
					}
					case HOUR:
					{							
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Hours--;
						if(RTC_TimeStructure.Hours>23) RTC_TimeStructure.Hours = 23; //because unsigned int, if it goes 'below' 0 it actually goes to 255,
																																					//so check if that overflow has happened and reset to 23
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Hours);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}	
					case MIN:
					{
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Minutes--;
						if(RTC_TimeStructure.Minutes>59) RTC_TimeStructure.Minutes = 59; //same as HOUR case, check if minutes is 'negative' (unsigned ints)
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Minutes);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}
					case SEC:
					{
						HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
						RTC_TimeStructure.Seconds--;
						if(RTC_TimeStructure.Seconds>59) RTC_TimeStructure.Seconds = 59;
						BSP_LCD_GLASS_Clear();
						sprintf((char*)timedate,"%d",RTC_TimeStructure.Seconds);
						BSP_LCD_GLASS_DisplayString(timedate);
						HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
						break;
					}
				}
				downpressed=0;
			}
			

			if (selpressed==1) {selpressed=0;} //ignore in DS mode
			if (leftpressed==1) 
			{
				dtSetting++; //go to next setting
				if (dtSetting == 7) {dtSetting=0;}
				Display_Curr_Setting(dtSetting);
				leftpressed=0;
			}
			if (rightpressed==1) 
			{
				dsmode=0; //turn off DateSet Mode
				RTC_AlarmA_IT_Enable(&RTCHandle);
				rightpressed=0;
			}
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

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

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

void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.

			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
					
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		//***********students: need to complete the following lines******************************
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
				
				RTCHandle.Init.AsynchPrediv = 0x7F; 
				RTCHandle.Init.SynchPrediv = 0xFF; 
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	
		//****3.***** init the time and date
 		//*****************Students: please complete the following lnes*****************************
				RTC_DateStructure.Year = 0x12;
				RTC_DateStructure.Month = RTC_MONTH_OCTOBER;
				RTC_DateStructure.Date = 0x17;
				RTC_DateStructure.WeekDay = RTC_WEEKDAY_TUESDAY;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
				RTC_TimeStructure.Hours = 0x0;  
				RTC_TimeStructure.Minutes = 0x02; 
				RTC_TimeStructure.Seconds = 0x00;
				RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to �1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	//**************students:  you need to set the following two lines****************
	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL; 
	  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

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
						selpressed=1;	
						RTC_AlarmA_IT_Disable(&RTCHandle);
						break;	
			case GPIO_PIN_1:     //left button						
						BSP_LCD_GLASS_Clear();
						RTC_AlarmA_IT_Disable(&RTCHandle);
						leftpressed=1;
						break;
			case GPIO_PIN_2:    //right button
						BSP_LCD_GLASS_Clear();
						RTC_AlarmA_IT_Disable(&RTCHandle);
						rightpressed=1;			
						break;
			case GPIO_PIN_3:    //up button							
						uppressed=1;
							break;
			case GPIO_PIN_5:    //down button						
						downpressed=1;
						break;
			case GPIO_PIN_14:   //external pushbutton 	
						RTC_AlarmA_IT_Disable(&RTCHandle);
						BSP_LCD_GLASS_Clear();
						pbpressed=1;
						break;			
			default://
						//default
						break;
	  } 
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  BSP_LED_Toggle(LED5);
	RTC_TimeShow();
}

void Display_Curr_Setting(enum Setting SET)
{
	switch (SET) {
		case YEAR: BSP_LCD_GLASS_DisplayString((uint8_t*)" YEAR");
			break;
		case MONTH: BSP_LCD_GLASS_DisplayString((uint8_t*)" MONTH");
			break;
		case WDAY: BSP_LCD_GLASS_DisplayString((uint8_t*)" WDAY");
			break;
		case DAY: BSP_LCD_GLASS_DisplayString((uint8_t*)" DAY");
			break;
		case HOUR: BSP_LCD_GLASS_DisplayString((uint8_t*)" HOUR");
			break;
		case MIN: BSP_LCD_GLASS_DisplayString((uint8_t*)" MIN");
			break;
		case SEC: BSP_LCD_GLASS_DisplayString((uint8_t*)" SEC");
			break;
	}
		
}

void RTC_TimeShow(void)
{
	BSP_LCD_GLASS_Clear();
  
  /* Get current RTC time & date */
  HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	//Format for display: hh:mm:ss
	sprintf((char*)timedate,"%d-%d-%d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
	BSP_LCD_GLASS_DisplayString((uint8_t*)timedate);
	
	//Format for scrolling with labels, but issues with LCD. Scrolling every second is super fast and unreadable
	//sprintf((char*)timedate,"%d-%d-%d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
	//BSP_LCD_GLASS_ScrollSentence((uint8_t*)timedate,1,200);
	
}

void RTC_DateDisplay(void)
{
	/* Get current RTC time & date */
  HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	//Format: weekday:day:month:year
	sprintf((char*)timedate," WDAY-%d:DAY-%d:MONTH-%d:YEAR-%02d",RTC_DateStructure.WeekDay, RTC_DateStructure.Date, RTC_DateStructure.Month, RTC_DateStructure.Year);
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)timedate,1,150);
}

void RTC_SavePressTime(void)
{
	//use this for saving current time to EEPROM - will need to implement with pushbutton
	/* Get current RTC time & date */
  HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	
	//Need hours, minutes, seconds - save into hh, mm, ss already declared
	hh = RTC_TimeStructure.Hours;
	mm = RTC_TimeStructure.Minutes;
	ss = RTC_TimeStructure.Seconds;

	EEPROM_SaveTime(hh, mm, ss);
}

void EEPROM_SaveTime(uint8_t hour, uint8_t min, uint8_t sec)
{
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"SAVING");
	
	//Save into EEPROM - treated sort of like a stack, save into 1st, 2nd, 3rd locations from current
	//Stack pointer (SP), then increment SP to point to new top of stack
	I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation+SP, hour);
	HAL_Delay(10);
	I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation+SP+1, min);
	HAL_Delay(10);
	I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation+SP+2, sec);
	HAL_Delay(500); //longer to display the 'SAVING' long enough for user to read
	SP +=3;
	I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation, SP);
	HAL_Delay(10);
}

void EEPROM_ReadTime(uint8_t SPoffset)
{
	//Hour, minute, second saved in previous 3 positions (plus offset if reading something other than latest save)
	uint8_t hourRead, minRead, secRead;
	hourRead=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+SP-3-SPoffset);
	HAL_Delay(10);
	minRead=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+SP-2-SPoffset);
	HAL_Delay(10);
	secRead=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+SP-1-SPoffset);
	HAL_Delay(10);
	BSP_LCD_GLASS_Clear();
	sprintf((char*)timedate," H%d-M%d-S%d",hourRead, minRead, secRead);
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)timedate,1,300);
	HAL_Delay(1000); //hold time on screen long enough
}

void Pushbutton_Init(void)
{
	//Enable clock
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	//Configure pins
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; //change to input
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN; //want pulldown to do external debouncing
	
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	//Set Interrupt Priority
	HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 0x0F, 0x00);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
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
