/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "Button.h"
#include "Tick.h"
#include "Thread.h"
#include "Dust.h"
#include "CJSON.h"
//#include "esp8266.h"
#include "fpe.h"
#include "stdlib.h"
#include "SIM808.h"
#include "control.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FRE_PER_PWM 20
#define FRE_ERROR 30
#define FRE_PWM_MAX_SHIFT 2
#define max(a,b) (((a)<(b))?(b):(a))
#define min(a,b) (((a)>(b))?(b):(a))
#define F_F_1 300  //设为600最好，因为此时不是风扇最低转速，但与最低转速相差不大，故用起来感觉一档转速差别不大；但是其转速差值（更高）是存在的，所以调速程序会起作用
									 //使其在一档时会让净化器度过风扇最初的“PWM波占空比升高不可调速区”，在1档转2档时风俗切换更快（最低档PWM占空比约36%）
#define F_F_2 750
#define F_F_3 1200
#define F_F_4 1650
#define F_F_5 2100
#define F_F_6 2600
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/**********************wifi***************************/
//#define getSetting(name) (name?name:name##Default)
//__IO uint32_t g_main_wifi_interval = 600;


//#define AUTO_POWEROFF 1
//#define DEBUG_MODE 0
//#define SYSTEM_AUTOPOFF_MS 600000
//#define RESET_WHILE_NO_DATEGET 60000

//#define FPE_UID_ADDRESS 0x00110000

//#define FPE_NEVERUSED_ADDRESS 0x00001000
//#define MAIN_STATUS_ADDRESS 0x00000100
//#define PM_RATIO_ADDRESS 0x00000200

//#define COMPRESS_BMP 0

////extern __IO uint8_t g_main_status;

//uint32_t g_main_lastWifiTick = RESET_WHILE_NO_DATEGET;
//uint32_t g_main_pairTick = 0;
//__IO uint8_t g_main_pairToDo = 1;
//char* g_main_udpAddressDefault = "www.hwlantian.com";//"172.28.50.1";//
//char* g_main_udpAddress = NULL;
//uint16_t g_main_udpPortDefault = 59830;
//uint16_t g_main_udpPort = 0;
////char* g_main_uidDefault = "M2-B2-1070";
//char* g_main_uidDefault = "M2";
//char* g_main_uid = NULL;
//__IO uint8_t g_main_ssidToDo = 1;
//__IO uint8_t g_main_neverUsed;
//uint32_t g_main_lastResponseTick = (UINT32_MAX - 5000);


//enum wifi_status
//{
//	WIFI_INIT = 0,
//	WIFI_SMARTING,
//	WIFI_REMOTE_ERROR,
//	WIFI_CONNECTED,
//	WIFI_RETRYING,
//	WIFI_NO_AP,
//	WIFI_SERVER_ACK
//};
//__IO uint8_t g_main_wifi_status = WIFI_NO_AP;


//void wifiSmartLinkStarted(void);
//void pairStart(void);
//void updateWifiStatus(void);
//static void updateStringSetting(char* value, char** pCurrent);
//void wifi_response(void);
//void mainThread(void);
/**************************/

/***************SIM808******************/
#define DEVICE_VERSION "JHQ"
#define getSetting(name) (name?name:name##Default)

char* g_main_udpAddressDefault = "123.57.56.40";//"172.28.50.1";//
char* g_main_udpAddress = NULL;
uint16_t g_main_udpPortDefault = 59830;
uint16_t g_main_udpPort = 0;

uint32_t g_main_uploadDataInterval = UPLOAD_DATA_INTERVAL_MS;

bool g_main_gps_updated = false;
bool g_main_pm25_stable = false;
bool g_main_startSleep = false;

static void updateStringSetting(char* value, char** pCurrent);
void sim808_HandleResponse(void);
void mainThread(void);
void check_sleep(void);

/**************************************/
extern dustvalue_t g_firPM ;//extern外部变量声明时，不需要再初始化

__IO int32_t g_FanFrequency = 600;     //1550<=g_FanFrequency<=5500,单位为(转/min) 
__IO int32_t fre_1;
__IO int32_t fre_2;
__IO int32_t fre_3;
__IO double fan = 600;
__IO int mode = 2;
__IO bool power = 1;
__IO bool LEDSwitch;
__IO bool buttonNoise;

//__IO ITStatus types = RESET;
//__IO uint32_t PM2D5=g_firPM.PM2D5V;   //单位为(1/100)ug/m3

enum STATUS_ID
{
	STATUS_MODULE= 0,
	STATUS_BUTTON=1,
	STATUS_WIFI=2,
  STATUS_POWEROFF=3,
//	STATUS_CONTROLED=4,
};
enum STATUS_ID last_g_main_status = STATUS_MODULE;
enum STATUS_ID g_main_status = STATUS_MODULE;
__IO uint8_t g_FanFrequency_wifi = 0;

enum BUTTON_IO
{
	BUTTON_DOME = 1,
	BUTTON_TOUCH = 2,
	BUTTON_NUMBER,
};
enum FAN_STATUS_ID
{
	STATUS_ONE_LEVEL = 0,
	STATUS_TWO_LEVEL = 1,
	STATUS_THREE_LEVEL = 2,
	STATUS_FOUR_LEVEL = 3,
	STATUS_FIVE_LEVEL = 4,
	STATUS_SIX_LEVEL = 5,

};

enum FAN_STATUS_ID FAN_STATUS =STATUS_ONE_LEVEL;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Fan_init(void);
uint8_t isButtonPressed(uint16_t id);
void LED_UNLIGNT(void);
//void onNomalButtonPressed(void ); //正常选择
//void onDoubleButtonPressed(void );//自动选档
void sim_update_examine(void);
//void onHoldButtonPressed(void );  //wifi配置
void onLongHoldButtonPressed(void );	//关机配置

//void auto_updata_click (void); //click状态下刷新转速和指示灯状态
void auto_updata_module (void);//模块自动调节状态下自动刷新转速和指示灯状态
void auto_updata_wifi(void);
void auto_updata_close(void);
void auto_updata_sim(void);

void fan_detect1(void);
void fan_detect2(void);
void fan_detect3(void);

//static void TransmitChar(char ch)
//{
//USART1->TDR = (ch & (uint8_t)0xFF);  
//while((USART1->ISR&UART_FLAG_TC)==0)
//;
//}
//void TransmitString(char * input)
//{
//while(1)
//{
//char ch = *(input++);
//if (ch == '\0')
//{
//break;
//}
//TransmitChar(ch);
//}
//}
//void TransmitDec(uint32_t input)
//{
//uint32_t quotient = 0;
//uint32_t divisor = 1000000000UL;
//uint8_t RightJustfy = 1;
//uint8_t showFirstZeros = 0;
//while(divisor>0)
//{
//quotient = (input)/divisor;
//if (quotient>0 || divisor == 1 || RightJustfy) 
//{
//char ch = quotient + '0';
//if ((!showFirstZeros && quotient == 0) && divisor != 1)
//{
//ch = 0x20;
//}
//else
//{
//showFirstZeros = 1;
//}
//TransmitChar(ch);
//input -= quotient*divisor;
//RightJustfy = 1;
//}
//divisor/=x10;
//}
//}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	tick_init(&htim14);
	thread_init(0,tick_ms());
	fpe_init(62,63); //初始化FLASH

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝，初始亮灯
	Fan_init();

	button_init(BUTTON_DOME,isButtonPressed);
//	button_addListener(BUTTON_DOME,BUTTON_STATUS_CLICKED_NO_NEXT,onNomalButtonPressed,0);//正常选择
//	button_addListener(BUTTON_DOME,BUTTON_STATUS_DOUBLE_CLICKED,onDoubleButtonPressed,0);//自动选档
//	button_addListener(BUTTON_DOME,BUTTON_STATUS_HOLD,onHoldButtonPressed,0);						 //wifi配置
	button_addListener(BUTTON_DOME,BUTTON_STATUS_LONG_HOLD,onLongHoldButtonPressed,0);	 //关机配置

	thread_quickAdd(0,button_run,10,0,0);//在程序运行的第一遍是不运行的，是由于execute函数貌似不运行
//	thread_quickAdd(0,auto_updata_click,100,0,0);
	thread_quickAdd(0,auto_updata_module,100,0,0);
//	thread_quickAdd(0,auto_updata_wifi,100,0,0);
	thread_quickAdd(0,auto_updata_close,100,0,0);
	thread_quickAdd(0,dust_update,1000,1,0);
	thread_quickAdd(0,sim_update_examine,1000,1,0);
	thread_quickAdd(0,auto_updata_sim,1000,1,0);
	
//	thread_quickAdd(0,wifi_response,1000,0,0);//检测是否有服务器传输回来文件（模块同时向主芯片传送），若此传输回来，通过CJSON中的格式解析JSON包，获取数据
//	thread_quickAdd(0,mainThread,1000,0,0);
//	thread_quickAdd(0,esp8266_run,10,1,0);
//	thread_quickAdd(0,updateWifiStatus,1000,1,0);//检查smart link的状态

	thread_quickAdd(0,sim808_HandleResponse,100,0,0); //接受并分析数据
	thread_quickAdd(0,sim808_UpdateStatus,1000,0,0);
	thread_quickAdd(0,mainThread,5000,0,0);
	thread_quickAdd(0,sim808_run,10,0,0);

	tick_add(button_tick,10);
	tick_add(fan_detect1,1);
	tick_add(fan_detect2,1);
	tick_add(fan_detect3,1);//原来认为他会卡住 

	tick_start();

	PM2D5_init();
	sim808_init(&huart2);//wait to init successfully.   
//	esp8266_init(&huart2);
//	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		thread_run(0,tick_ms(),0,0);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void Fan_init(void)
{
		if(g_main_status!=3)
		{
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//打开风扇
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		}
}
uint8_t isButtonPressed(uint16_t id)//返回1和0,1表示按下，0表示未按下。所以当按键正常为1而按下为0时，应该取反
{																		//通过 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)确定按键的按下
	switch(id)
	{
		case BUTTON_DOME:
			return HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12);
		case BUTTON_NUMBER:
			return 0;
		case BUTTON_TOUCH:
			return 0;
	}
	return 0;
}
void LED_UNLIGNT()    //全灭LED灯
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);  //LED3 蓝
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_1);  //LED1 绿
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_2);  //LED2 红
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);  //LED6 蓝
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2); //LED4 绿
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3); //LED5 红
//	HAL_TIM_PWM_Stop(&htim17,TIM_CHANNEL_1);  //LED9 蓝
//	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);  //LED7 绿
//	HAL_TIM_PWM_Stop(&htim16,TIM_CHANNEL_1);  //LED8 红
}

//void onNomalButtonPressed()      //正常选择
//{
//	if (g_main_status==1)                    //只有在为正常按键模式下，才会有响应
//	{
//		if (FAN_STATUS<5)
//			FAN_STATUS++;
//		else
//			FAN_STATUS=STATUS_ONE_LEVEL;
//	}
//}

//void onDoubleButtonPressed( )     	  //自动选档
//{																
////	status_change(1);
//	if(g_main_status<2)//若小于等于2，则会使g_main_status存在=3的状态
//		{	g_main_status++;
//			g_FanFrequency=F_F_1;
//		}
//	else if (g_main_status==2)
//		{	g_main_status=STATUS_MODULE;
//			g_FanFrequency=F_F_1;
//		}
//}

void sim_update_examine(void)
{
	if(power == 0)
	{
		g_main_status = STATUS_POWEROFF;
	}
	else if(power == 1)
	{
		if(mode == 1)
		{
			g_main_status = STATUS_BUTTON;
		}
		else if (mode == 2)
		{
			g_main_status = STATUS_MODULE;
		}
		
		
	}	
}


//void onHoldButtonPressed(void ) //wifi配置
//{
////	status_change(2);
//	if (g_main_status == 2)//
//	{
////		g_FanFrequency = 0; //使转速为零，确保wifi的状态监测及处理不会在wifi smart link状态下亮表示转速的灯
//		esp8266_smart(60000);	//只是配置wifi，与pair配对是完全不同的两个过程；按照方式3模块通讯，若通讯成功，则置wifi配置位为1.g_esp8266_smartTick是用来作为超时限定的
//													//注意：与pair配对无关
//		updateWifiStatus();   //可删去，只是起连通wifi后迅速刷新状态的作用	
//	}
//}

void onLongHoldButtonPressed(void)
{
		if(g_main_status!=3)
		{	
			last_g_main_status=g_main_status;
			g_main_status=STATUS_POWEROFF;
		}
		else
		{
			g_main_status = last_g_main_status;
		}
}

void auto_updata_module (void)
{
//	HAL_GetTick();
	if(g_main_status == 0)	         	//模块自动档函数持续运行，直到再次双击使g_main_status改变
	{																	//锁定为模块自动调节状态，单独按一下无响应（状态互换加上状态选取，选取在互换后为1的状态运行模块自动档函数）
		LED_UNLIGNT();                  //只有在id为1的情况，也就是双击模式开启的情况下，每次更新LED状态前都有重置一下
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//打开风扇
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		if (g_firPM.PM2D5V<=3500)
			{
				g_FanFrequency=F_F_1;
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
			}
		else if(g_firPM.PM2D5V<=7500 &&g_firPM.PM2D5V>3500)
			{
				g_FanFrequency=F_F_2;
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED6 蓝
			}
		else if(g_firPM.PM2D5V<=11500 && g_firPM.PM2D5V>7500)  
			{
				g_FanFrequency=F_F_3;
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
			}
		else if(g_firPM.PM2D5V<=15000 && g_firPM.PM2D5V>11500) 
			{
				g_FanFrequency=F_F_4;
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //LED4 绿
			}
		else if(g_firPM.PM2D5V<=25000 && g_firPM.PM2D5V>15000) 
			{
				g_FanFrequency=F_F_5;
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
			}
		else 
			{
				g_FanFrequency=F_F_6;
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //LED5 红
			}
	}
}

//void auto_updata_click (void)
//{
//	if (g_main_status==1)                    //只有在为正常按键模式下，才会有响应
//	{
//		LED_UNLIGNT();
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//打开风扇
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);

//		switch(FAN_STATUS)
//		{
//			case 0:g_FanFrequency=F_F_1;	
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
//							break;
//			case 1:g_FanFrequency=F_F_2;
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED6 蓝
//							break;
//			case 2:g_FanFrequency=F_F_3;
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
//							break;
//			case 3:g_FanFrequency=F_F_4;
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //LED4 绿
//							break;
//			case 4:g_FanFrequency=F_F_5;
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
//							break;
//			case 5:g_FanFrequency=F_F_6;
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //LED5 红
//							break;
//		}
//	}
//}


//void auto_updata_wifi(void)
//{
//	if(g_main_status == 2)
//	{
//		LED_UNLIGNT();
//		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//打开风扇
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);//这里若不使控制引脚（control）置一，则其为浮空，不能使风扇转动
//		g_FanFrequency == g_FanFrequency_wifi;
//		
//		switch(g_FanFrequency)
//		{
//			case F_F_1:	
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
//							break;
//			case F_F_2:
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED6 蓝
//							break;
//			case F_F_3:
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
//							break;
//			case F_F_4:
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //LED4 绿
//							break;
//			case F_F_5:
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
//							break;
//			case F_F_6:
//							HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
//							HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //LED5 红
//							break;
//		}
//	}
//}

void auto_updata_close(void)
{
	if(g_main_status == 3)
	{
		LED_UNLIGNT();
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
	}
}

void auto_updata_sim(void)
{
	if(g_main_status == STATUS_BUTTON)
	{
		LED_UNLIGNT();
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);//打开风扇
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);//这里若不使控制引脚（control）置一，则其为浮空，不能使风扇转动
		g_FanFrequency = fan;
		
		if (g_FanFrequency<=F_F_1)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
			}
		else if(g_FanFrequency<=F_F_2)
			{
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED6 蓝
			}
		else if(g_FanFrequency<=F_F_3)  
			{
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
			}
		else if(g_FanFrequency<=F_F_4) 
			{
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1); //LED1 绿
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //LED4 绿
			}
		else if(g_FanFrequency<=F_F_5) 
			{
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
			}
		else 
			{
				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2); //LED2 红
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //LED5 红
			}
	}
}

void fan_detect1()
{
static uint32_t all = 0;
static uint32_t times = 0;
static uint16_t lasthigh = 0;
uint16_t high = 0;
	
if((GPIOA->IDR & GPIO_PIN_15) == 0)
{
	
	high = 1;
}
else
{
	high = 0;
}
all ++;
if(high != lasthigh )
{
	lasthigh = high;  //从这里lasthign得到了上一个hign的值
	times ++;                  //若想知道程序运行处的时间，可以看TIM1定时器的CNT寄存器，由于TIM1的预分频器PSC=47，则可计算为1S/48M*(47+1)=1us
														 //即CNT为程序运行微秒数
}
if(times == 8 || all > 2000)//此处接收反馈回来的信号，times在反馈信号发生一次转变时加一，即以此时的风扇频率初始状态50%——108Hz为例，
														//一次反转时间为1S/108/2=4.6ms；而all值取决于：tick_add(fan_detect1,1)中的1代表100us（由tim14的47、99可知），也就是说一次检测耗时100us；
														//这里的all是8次转换的最多耗时，由示波器可知在最低档，8次转换最多耗时200ms，也就是200*1000/100=2000次检测，故all=2000；
{
	//int32_t fre = times*UINT32_C(500)/all;//所测出的频率与实际输入频率比值约为5
	fre_1 = times*UINT32_C(150000)/all;//公式的确定是由：反馈波一次转换耗时=all*100/times  ==>  风扇转一周耗时=all*100*4/times  ==》  风扇60秒转的周数=60*1 000 000/风扇转一周耗时
																	 //===>>fre=150000*times/all

	times = 0;
	all = 0;
	__IO int32_t deltaFre = (int32_t)g_FanFrequency - fre_1;//为了达到所需要的频率g_FanFrequency,需要从实际频率fre改变的频率值为deltaFre
//	static int32_t deltaFre_last = 0;
	__IO int32_t ccr = TIM3->CCR1;                            //而g_FanFrequency为要求的风扇转速（转/min），即*2/60
	__IO int32_t total = TIM3->ARR;
//	TransmitDec(fre);
//	TransmitChar(',');	
//	TransmitDec(ccr);
//	TransmitChar('\n');
//deltaFreSum += deltaFre;
//	if (abs(deltaFre)>50)
//	{
//	ccr += ( deltaFre * 1  ) /FRE_PER_PWM ;
//	}
//	deltaFreLast = deltaFre;
//	ccr = 459;
//	else
//	{
//	ccr += ( deltaFre * 1 + deltaFreSum*0.01) /FRE_PER_PWM ;
//	}
	
	if(deltaFre < -FRE_ERROR)
	{
		
		//deltaFre +=FRE_PER_PWM - FRE_ERROR;
//		if ((deltaFre)>-200)
//		{
//		  int32_t KdErr = deltaFre + deltaFre_last;
//		ccr +=max((deltaFre+KdErr)/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
			ccr +=max(deltaFre/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
//		}
//		else
//		{
//		ccr +=max(deltaFre/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));
//		}
		
//		ccr -=min(deltaFre/FRE_PER_PWM-1,(total>>FRE_PWM_MAX_SHIFT));
		// deltaFre +=49;
		// ccr +=deltaFre/50;
	}
	else if (deltaFre > FRE_ERROR)
	{
//		if ((deltaFre)<200)
//		{
//		 int32_t KdErr = deltaFre + deltaFre_last;
			ccr +=min((deltaFre)/FRE_PER_PWM,total>>FRE_PWM_MAX_SHIFT);
//		}
//		else
//		{
//			__IO int32_t a = (deltaFre)/FRE_PER_PWM;
//			__IO int32_t b = (total>>FRE_PWM_MAX_SHIFT);
//			ccr +=min(a,b);
//		}
		//deltaFre -=FRE_PER_PWM - FRE_ERROR;
	//	ccr +=min(-deltaFre/FRE_PER_PWM,total>>FRE_PWM_MAX_SHIFT);
		// deltaFre -=40;
		// ccr +=deltaFre/50;
	}
//	deltaFre_last = deltaFre;
	ccr = max(0,min(total,ccr));
	TIM3->CCR1= ccr ;
	}
}

void fan_detect2()
{
static uint32_t all = 0;
static uint32_t times = 0;
static uint16_t lasthigh = 0;
uint16_t high = 0;
	
if((GPIOB->IDR & GPIO_PIN_3) == 0)
{
	
	high = 1;
}
else
{
	high = 0;
}
all ++;
if(high != lasthigh )
{
	lasthigh = high;  //从这里lasthign得到了上一个hign的值
	times ++;                  //若想知道程序运行处的时间，可以看TIM1定时器的CNT寄存器，由于TIM1的预分频器PSC=47，则可计算为1S/48M*(47+1)=1us
														 //即CNT为程序运行微秒数
}
if(times == 8 || all > 2000)//此处接收反馈回来的信号，times在反馈信号发生一次转变时加一，即以此时的风扇频率初始状态50%——108Hz为例，
														 //一次反转时间为1S/108/2=4.6ms；而all在
{
	fre_2 = times*UINT32_C(150000)/all;
	times = 0;
	all = 0;
	// int32_t targetFre = 4000;
	__IO int32_t deltaFre = (int32_t)g_FanFrequency - fre_2 ;//为了达到所需要的频率g_FanFrequency,需要从实际频率fre改变的频率值为deltaFre
//	static int32_t deltaFre_last = 0;
	__IO int32_t ccr = TIM3->CCR2 ;                            //而g_FanFrequency为要求的风扇转速（转/min），即*2/60
	__IO int32_t total = TIM3->ARR;
	
	if(deltaFre < -FRE_ERROR)
	{
//		  int32_t KdErr = deltaFre + deltaFre_last;
//		  ccr +=max((deltaFre+KdErr)/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
			ccr +=max(deltaFre/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
	}
	else if (deltaFre > FRE_ERROR)
	{
//		 int32_t KdErr = deltaFre + deltaFre_last;
		 ccr +=min((deltaFre)/FRE_PER_PWM,total>>FRE_PWM_MAX_SHIFT);
	}
	ccr = max(0,min(total,ccr));
	TIM3->CCR2 = ccr ;
	}
}

void fan_detect3()
{
static uint32_t all = 0;
static uint32_t times = 0;
static uint16_t lasthigh = 0;
uint16_t high = 0;
	
if((GPIOB->IDR & GPIO_PIN_9) == 0)
{
	
	high = 1;
}
else
{
	high = 0;
}
all ++;
if(high != lasthigh )
{
	lasthigh = high;  //从这里lasthign得到了上一个hign的值
	times ++;                  //若想知道程序运行处的时间，可以看TIM1定时器的CNT寄存器，由于TIM1的预分频器PSC=47，则可计算为1S/48M*(47+1)=1us
														 //即CNT为程序运行微秒数
}
if(times == 8 || all > 2000)//此处接收反馈回来的信号，times在反馈信号发生一次转变时加一，即以此时的风扇频率初始状态50%——108Hz为例，
														 //一次反转时间为1S/108/2=4.6ms；而all在
{
	fre_3 = times*UINT32_C(150000)/all;
	times = 0;
	all = 0;
	__IO int32_t deltaFre = (int32_t)g_FanFrequency - fre_3;//为了达到所需要的频率g_FanFrequency,需要从实际频率fre改变的频率值为deltaFre
//	static int32_t deltaFre_last = 0;
	__IO int32_t ccr = TIM16->CCR1 ;                            //而g_FanFrequency为要求的风扇转速（转/min），即*2/60
	__IO int32_t total = TIM16->ARR;
	
	if(deltaFre < -FRE_ERROR)
	{
//		  int32_t KdErr = deltaFre + deltaFre_last;
//		  ccr +=max((deltaFre+KdErr)/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
			ccr +=max(deltaFre/FRE_PER_PWM,-(total>>FRE_PWM_MAX_SHIFT));//-(total>>FRE_PWM_MAX_SHIFT)限制风扇一次调速不可超过1/4个ARR
	}
	else if (deltaFre > FRE_ERROR)
	{
//		 int32_t KdErr = deltaFre + deltaFre_last;
		 ccr +=min((deltaFre)/FRE_PER_PWM,total>>FRE_PWM_MAX_SHIFT);
	}
	ccr = max(0,min(total,ccr));
	TIM16->CCR1 = ccr ;
	}
}


//void updateWifiStatus(void)
//{
//	if (g_main_status == STATUS_POWEROFF) //POWEROFF是 是表示切断电源应属于枚举类型STATUS_ID的一员，而不是像onPowerButtonPressed这个函数一样相应wifi的启动
//	{
//		return;
//	}
//	uint8_t status = esp8266_getStatus();
//if (g_main_status == STATUS_WIFI)
//{
//	LED_UNLIGNT();
//	switch(status)
//	{
//		case 1: //smart link
//		{
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //LED3 蓝
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //LED6 蓝
//		//	setLedList(LED_LIST_RAINBOW,sizeof(LED_LIST_RAINBOW)>>2,5000);
//			g_main_wifi_status = WIFI_SMARTING;
//			break;
//		}
//		case 3: //connceted		
//		case 4: //unconncet
//		{
//			
//			if (g_main_lastResponseTick+5000 - tick_ms() < (UINT32_MAX>>1))//服务器最后回复模块的时间与现在的时间相差不到5秒，即5000ms；随后回复时刻加上五秒减去现在时刻若为正值，则正常，
//																																		 //	若为负值，其补码为无穷大，故这样<(UINT32_MAX>>1)
//			{
////				if (g_main_requestLedList && g_main_requestLedListSize && g_main_requestLedListTime)
////				{
////					setLedList(g_main_requestLedList,g_main_requestLedListSize,g_main_requestLedListTime);
////				}
////				else
////				{
////					setLedList(LED_LIST_GREEN_BREATH,sizeof(LED_LIST_GREEN_BREATH)>>2,3000);
////				}
//				g_main_wifi_status = WIFI_SERVER_ACK;
//				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);  //LED1 绿
//				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);  //LED2 红
//				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);   //LED3 蓝
//				
//				if ( g_main_neverUsed )
//				{
//					fpe_write(FPE_NEVERUSED_ADDRESS,0);
//					g_main_neverUsed = 0;
//				}
//			}
//			else 
//			{
//				if (status == 3)
//				{

//				g_main_wifi_status = WIFI_CONNECTED;
//				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);  //LED1 绿
//				}
//				else
//				{
//				g_main_wifi_status = WIFI_REMOTE_ERROR;
//				HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);  //LED2 红
//				}
//				//setLedList(LED_LIST_YELLOW_BLINK,sizeof(LED_LIST_YELLOW_BLINK)>>2,200);
//				
//			}
//			break;
//		}
//		case 5: //no wifi
//		{
//			g_main_wifi_status = WIFI_NO_AP;
//			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); //LED5 红
//			//setLedList(LED_LIST_BLUE_BLINK,sizeof(LED_LIST_BLUE_BLINK)>>2,200);
//			break;
//		}
//		case 2: //no wifi
//		{
//			g_main_wifi_status = WIFI_REMOTE_ERROR;;
//			//setLedList(LED_LIST_BLUE_BLINK,sizeof(LED_LIST_BLUE_BLINK)>>2,200);
//			break;
//		}
//	}
//}
//}
//static void updateStringSetting(char* value, char** pCurrent)
//{
//	if (*pCurrent)
//	{
//		free(*pCurrent);
//	}
//	if (strlen(value) == 0)
//	{
//		*pCurrent = NULL;
//	}
//	else
//	{
//		char* copy = malloc(strlen(value)+1);
//		if (copy)
//		{
//			strcpy(copy,value);
//			*pCurrent = copy;
//		}
//	}
//}

//void wifi_response(void)                     //解析wifi模块向板子发送的CJSON包
//{
//	if (g_main_status == STATUS_POWEROFF)
//	{
//		return;
//	}
//	char* receive = esp8266_getReceiveString();
//	if (!receive)
//	{
//		if(tick_ms() - g_main_lastWifiTick < (UINT32_MAX>>1))
//		{
//			if (!esp8266_smarting())//
//			{
//				esp8266_reset();
//				g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//			}
//		}
//	}
//	else
//	{
//		g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//	}
//	if (receive)
//	{
//		cJSON* json = cJSON_Parse(receive);
//		if(json)
//		{
//			g_main_lastResponseTick = tick_ms();
//			cJSON* child;
//			child = cJSON_GetObjectItem(json,"udpAddress");
//			if (child)
//			{
//				updateStringSetting(child->valuestring, &g_main_udpAddress);
//			}
//			
//			child = cJSON_GetObjectItem(json,"udpPort");
//			if (child)
//			{
//				g_main_udpPort = child->valueint;
//			}
////			/************接收从WIFI模块传输过来的风扇转速************/
////			child = cJSON_GetObjectItem(json,"fanFrequency");          //得出风扇转速，wifi应该在传输中断后风扇转速不再更新
////			if (child)
////			{
////				g_FanFrequency = child->valueint;
////			}
////			/*****************************/
//			child = cJSON_GetObjectItem(json,"setUid");
//			if (child)
//			{
//				updateStringSetting(child->valuestring, &g_main_uid);
//				char* uid = child->valuestring;
//				fpe_writeString(FPE_UID_ADDRESS,uid);//记录UID到单片机内存中，再次启动就提取就行
//			}
//			if (g_main_pairToDo)
//			{
//				child = cJSON_GetObjectItem(json,"pairConfirmed");
//				if (child)
//				{
//					int16_t confirmed = child->valueint;
//					int16_t pairTick = tick_ms() - g_main_pairTick;
//					if (abs(confirmed-pairTick)<1000)
//					{
//						g_main_pairToDo = 0;
//					}
//				}
//			}
//			if (g_main_ssidToDo)
//			{
//				child = cJSON_GetObjectItem(json,"ssidConfirmed");
//				if (child)
//				{
//					char* confirmed = child->valuestring;
//					char* ssid = esp8266_getSSID();
//					if (strcmp(confirmed,ssid) == 0)
//					{
//						g_main_ssidToDo = 0;
//					}
//				}
//			}
//			
//						child = cJSON_GetObjectItem(json,"setInterval");
//			uint32_t interval = 0;
//			if (child)
//			{
//				interval = child->valueint;//1 to 10s
//			}	
//			if (interval>=1000)
//			{
//				if (interval != g_main_wifi_interval)// || g_main_wifi_status != WIFI_CONNECT)WIFI_CONNECTED
//				{
//					g_main_wifi_interval = interval;
//					thread_remove(0,mainThread);
//					thread_t* t = (thread_t*) malloc(sizeof(thread_t));
//					t->priority = 0;
//					t->remainTimes = 0;
//					t->executeTick = tick_ms()+g_main_wifi_interval;
//					t->function = mainThread;
//					t->intervalTick = g_main_wifi_interval;
//					t->minNeedTick = 0;
//					t->next = NULL;
//					if (thread_add(0,t,esp8266_run) == 0)
//					{
//						free(t);
//					}
//				//	g_main_wifi_getDelay = 5000;
//				//	g_main_wifi_triggerTick = tick_ms()+g_main_wifi_interval+g_main_wifi_getDelay;
//				}
//			}
//			cJSON_Delete(json);
//		}
//		free(receive);
//	}
//}

//void wifiSmartLinkStarted(void)
//{
//	g_main_ssidToDo = 1;
////	pairStart();
//}

//void pairStart(void)
//{
//	g_main_pairToDo = 1;
//	g_main_pairTick = tick_ms();

//}

//void mainThread(void)
//{
//	if (g_main_status == STATUS_POWEROFF)
//	{
//		return;
//	}
//	//th_update();
//	cJSON* json = cJSON_CreateObject();																	//SSID为密码，此处密码验证，建立CJSON包
//	if (json)
//	{																				
//		cJSON_AddStringToObject(json,"devId",getSetting(g_main_uid));
////		cJSON_AddStringToObject(json,"ver",DEVICE_VERSION);          //应该是显示相关的
////		if (g_main_power)
////		{
//			cJSON_AddNumberToObject(json,"pm2d5",0+g_firPM.PM2D5V/100);
//			cJSON_AddNumberToObject(json,"pm1d0",0+g_firPM.PM1D0V/100);
//			cJSON_AddNumberToObject(json,"pm10",0+g_firPM.PM10V/100);
////			cJSON_AddNumberToObject(json,"ch2o",0);
//////			cJSON_AddNumberToObject(json,"temp",g_main_temp/100.0);
//////			cJSON_AddNumberToObject(json,"hum",g_main_humidity/100.0);
////		cJSON_AddNumberToObject(json,"check",atoi(g_main_uid + strlen(g_main_uid)-2));//+g_main_PM2_5Value/100.0,这里应该只是上传，主要是uid上传，不需要更改太多
////		}
//		char* ssid = esp8266_getSSID();//这里上传SSID是为了让服务器对比模块上传的SSID和手机上传的SSID,来达到只pair相同的SSID下pairtick相差5秒的一对，但如果是两个局域网叫的名字一样呢
//		if (ssid)											
//		{
//			cJSON_AddStringToObject(json,"ssid",ssid);
//		}
//		if (g_main_pairToDo)//这里是模块与手机的配对，由于mainThread函数1秒运行一次，而pairTick > 120000是2分钟，所以这个函数的运行情况是不断变化的当pairtick大于两分钟时，
//		{										//停止pair(置g_main_pairToDo为0)，并且从发送的CJSON包中去掉"pair"字符，服务器不再进行配对
//			uint32_t pairTick = tick_ms() - g_main_pairTick;
//			if (pairTick > 120000)
//			{
//				g_main_pairToDo = 0;
//			}
//			else
//			{
//				cJSON_AddNumberToObject(json,"pair",pairTick);
//			}
//		}
//		cJSON_AddNumberToObject(json,"systemTick",tick_ms());
//		char * str = cJSON_PrintUnformatted(json);  ///从这里起是在干什么？？？cJSON包转化为字符串，模块只能接受到字符串
//		cJSON_Delete(json);//cJSON包转化为字符串后删除cJSON包
//		if (ssid)//ssid每次都要发给服务器，以确定手机模块在同一个wifi下，手机才能拿到数据
//		{
//			free(ssid);
//		}
//		if (str)
//		{
//			uint16_t length = strlen(str);
//			char* buffer = malloc(length+2);
//			if (buffer)
//			{
//				sprintf(buffer,"%s\n",str);
//				char* address = getSetting(g_main_udpAddress);//?g_main_udpAddress:g_main_udpAddressDefault;不存在就使用默认的值
//				uint16_t port = getSetting(g_main_udpPort);//g_main_udpPort!=0?g_main_udpPort:g_main_udpPortDefault;
//				esp8266_udp(buffer,address,port);
//				free(buffer);
//			}
//			free(str);
//		}
//	}
//	
//}

static void updateStringSetting(char* value, char** pCurrent)
{
	if (*pCurrent)
	{
		free(*pCurrent);
	}
	if (strlen(value) == 0)
	{
		*pCurrent = NULL;
	}
	else
	{
		char* copy = malloc(strlen(value)+1);
		if (copy)
		{
			strcpy(copy,value);
			*pCurrent = copy;
		}
	}
}


void sim808_HandleResponse(void)
{
	char* receive = sim808_getReceiveString();
//	if (!receive)
//	{
//		if(tick_ms() - g_main_lastWifiTick < (UINT32_MAX>>1))
//		{
//			if (!esp8266_smarting())
//			{
//				esp8266_reset();
//				g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//			}
//		}
//	}
//	else
//	{
//		g_main_lastWifiTick = tick_ms() + RESET_WHILE_NO_DATEGET;
//	}
	if (receive)////////////////////////111111111111111
	{
		cJSON* json = cJSON_Parse(receive);
		if(json)
		{
//			g_main_lastResponseTick = tick_ms();
			
			cJSON* status;
			status = cJSON_GetObjectItem(json,"status");
			if (status)
			{
				cJSON* child;
				
				child = cJSON_GetObjectItem(status,"udpAddress");
				if (child)
				{
					updateStringSetting(child->valuestring, &g_main_udpAddress);
				}
				
				child = cJSON_GetObjectItem(status,"udpPort");
				if (child)
				{
					g_main_udpPort = child->valueint;
				}
				
				child = cJSON_GetObjectItem(status,"power");
				if (child)
				{
					power = child->valueint;
				}
				
				child = cJSON_GetObjectItem(status,"mode");
				if (child)
				{
					mode = child->valueint;
				}
				
				child = cJSON_GetObjectItem(status,"fan");
				if (child)
				{
					fan = child->valueint;
				}
				
//				child = cJSON_GetObjectItem(status,"setInterval");
//				uint32_t interval = 0;
//				if (child)
//				{
//					interval = child->valueint;//1 to 10s
//				}	
//				if (interval>1000)
//				{
//	//				interval = interval*SERVER_INTERVAL_MULTIPLE; //server unit (s) to  local unit (ms)
//					if (interval != g_main_uploadDataInterval)// || g_main_wifi_status != WIFI_CONNECT)
//					{
//						g_main_uploadDataInterval = interval;
//						fpe_write(ADDRESS_UPLOAD_INTERVAL_VALUE, ( interval ) );
//						fpe_write(ADDRESS_UPLOAD_INTERVAL_ENSURE, FLAG_FLASH_DATA_ENSURE);
//					
//					//	g_main_wifi_getDelay = 5000;
//					//	g_main_wifi_triggerTick = tick_ms()+g_main_uploadDataInterval+g_main_wifi_getDelay;
//					}
//				}
//				cJSON_Delete(status);
			}
			cJSON_Delete(json);
		}
//		g_main_startSleep = true;	//set system to sleep mode
		free(receive);
	}
}

void mainThread(void)
{
	
	sim808_Info_t sim808_Info = sim808_GetInfo();

//		( (g_main_pm25_stable || tick_ms()>MAX_PM25_MS)	&& ( g_main_gps_updated || tick_ms()>MAX_GPS_MS) )
//	if( !(//wait gps new data until timeout,when time out,we use data in flash if exist
//		( (g_main_pm25_stable || tick_ms()>MAX_PM25_MS)	&& ( sim808_Info.gpsUpdated || tick_ms()>MAX_GPS_MS) )\
//	||(tick_ms()>MAX_WAKE_MS -20000)
//	) )
//	{
//		return;
//	}
	
	//th_update();
	
	cJSON* json = cJSON_CreateObject();
	if (json)
	{
		cJSON_AddStringToObject(json,"devId",sim808_Info.IMEI_number);
		cJSON_AddStringToObject(json,"ver",DEVICE_VERSION);
		
		cJSON_AddNumberToObject(json,"pm2d5",(double)(g_firPM.PM2D5V/100.0));
		cJSON_AddNumberToObject(json,"mode",(int)mode);
		cJSON_AddNumberToObject(json,"fan_1",(double)fre_1);
		cJSON_AddNumberToObject(json,"fan_2",(double)fre_2);
		cJSON_AddNumberToObject(json,"fan_3",(double)fre_3);
		cJSON_AddNumberToObject(json,"power",(bool)power  );
//		cJSON_AddNumberToObject(json,"LEDSwitch",(bool)  );
//		cJSON_AddNumberToObject(json,"buttonNoise",(bool)  );
		
		cJSON_AddNumberToObject(json,"systemTick",tick_ms());
		
		char * str = cJSON_PrintUnformatted(json);
		cJSON_Delete(json);

		if (str)
		{
			uint16_t length = strlen(str);
			char* buffer = malloc(length+3);//20160808
			if (buffer)
			{
				sprintf(buffer,"%s\n%c",str,0x1a);//20160808
				char* address = getSetting(g_main_udpAddress);//?g_main_udpAddress:g_main_udpAddressDefault;
				uint16_t port = getSetting(g_main_udpPort);//g_main_udpPort!=0?g_main_udpPort:g_main_udpPortDefault;
				sim808_udp(buffer,address,port);
				free(buffer);
			}
			free(str);
		}
	}
	
}

void do_sleep(void)
{	
//		disable_pm2d5();
		sim808_deinit();
		HAL_Delay(100);                                       //2015-12-7
		HAL_NVIC_SystemReset();		
}


void check_sleep(void)
{
	if (g_main_startSleep)//tickms超过该时间时会自动进入睡眠模式, 即使未完成整个流程.
	{
//		if(g_main_battery_voltage < RANGE_RUNTIME_BATTERY_LOW)
//		{
//			fpe_write(ADDRESS_BATTERY_RANGE,RANGE_START_BATTERY_LOW);  //refer to 'charge code', locating beginning of 'main()'function, about line 169.
//		}
		do_sleep();
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
