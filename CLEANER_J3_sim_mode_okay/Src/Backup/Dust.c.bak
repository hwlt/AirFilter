#include "Dust.h"

#define UART2_BUFFER_SIZE 64
#define UART2_BUFFER_MASK (UART2_BUFFER_SIZE - 1)
uint8_t g_uart2Buffer[UART2_BUFFER_SIZE] = {0};

dustvalue_t g_firPM ={.PM1D0V=0,.PM2D5V=0,.PM10V=0,.PC0D3V=0,.PC1D0V=0,.PC2D5V=0,.PC5D0V=0,.PC10V=0};//加点表示初始化的变量属于dustvalue_t型的结构体g_firPM，并且防止乱序
uint32_t PMnumber1_0,PMnumber2_5,PMnumber10;

void UART2_Config_B2(char *stringToSend)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)stringToSend,strlen(stringToSend),100);
}

void PM2D5_init(void)
{
	memset(g_uart2Buffer,UINT8_MAX-1,sizeof(g_uart2Buffer));//为何这样复位数组g_uart2Buffer,UINT8_MAX-1=254(1111 1101)?????
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)g_uart2Buffer,UART2_BUFFER_SIZE);
	UART2_Config_B2("at+outtype=8\r\n");//设置传输协议为8
	HAL_Delay(2);
	UART2_Config_B2("at+fco\r\n");
	HAL_Delay(2);
}

bool get_dust_data(void) //选定模块来接收数据
{
	UART2_Config_B2("at+getdata\r\n");
	HAL_Delay(20);
	__IO uint8_t tbuffer[UART2_BUFFER_SIZE] = {0};
	__IO uint8_t start = 0;
	for(uint16_t j = 0; j< UART2_BUFFER_SIZE ; j++)
	{
		uint16_t c = g_uart2Buffer[j];
		if(c == 0x5a && g_uart2Buffer[(j+1)&UART2_BUFFER_MASK] == 0xa5 && g_uart2Buffer[(j+2)&UART2_BUFFER_MASK] == 0x34)
		{
			start = j;
			__IO uint16_t check = 0;                  
			for(uint8_t k = 0; k<UART2_BUFFER_SIZE ; k++)
			{
				tbuffer[k] = g_uart2Buffer[(k+start) & UART2_BUFFER_MASK];
			}
			check = 0;
			for(uint16_t l = 0;l<56 ;l+=4)
			{
				check += tbuffer[l] + tbuffer[l+1] + (tbuffer[l+2]<<8) + (tbuffer[l+3]<<16);
			}
			if(((tbuffer[58]<<8)+tbuffer[59]) == check)
			{
//				uint32_t PMnumber1_0,PMnumber2_5,PMnumber10;
				uint32_t PCnumber0_3,PCnumber1_0,PCnumber2_5,PCnumber5_0,PCnumber10; 
				PMnumber1_0 = fmax(0,(tbuffer[11]<<24) +(tbuffer[10]<<16) +(tbuffer[9] <<8) + (tbuffer[8]));
				PMnumber2_5 = fmax(0,(tbuffer[19]<<24) +(tbuffer[18]<<16) +(tbuffer[17]<<8) + (tbuffer[16]));
				PMnumber10  = fmax(0,(tbuffer[35]<<24) +(tbuffer[34]<<16) +(tbuffer[33]<<8) + (tbuffer[32]));
				
				PCnumber0_3 = fmax(0,(tbuffer[39]<<24) +(tbuffer[38]<<16) +(tbuffer[37]<<8) + (tbuffer[36]));
				PCnumber1_0 = fmax(0,(tbuffer[43]<<24) +(tbuffer[42]<<16) +(tbuffer[41]<<8) + (tbuffer[40]));
				PCnumber2_5 = fmax(0,(tbuffer[47]<<24) +(tbuffer[46]<<16) +(tbuffer[45]<<8) + (tbuffer[44]));
				PCnumber5_0 = fmax(0,(tbuffer[51]<<24) +(tbuffer[50]<<16) +(tbuffer[49]<<8) + (tbuffer[48]));
				PCnumber10  = fmax(0,(tbuffer[55]<<24) +(tbuffer[54]<<16) +(tbuffer[53]<<8) + (tbuffer[52]));
				
				{
					g_firPM.PM1D0V = fmin(PMnumber1_0,99999);
					g_firPM.PM2D5V = fmin(PMnumber2_5,99999);
					g_firPM.PM10V  = fmin(PMnumber10,99999);
					
					g_firPM.PC0D3V = PCnumber0_3;
					g_firPM.PC1D0V = PCnumber1_0;
					g_firPM.PC2D5V = PCnumber2_5;
					g_firPM.PC5D0V = PCnumber5_0;
					g_firPM.PC10V  = PCnumber10;
				}
				return true;
			}
			return false;
		}
		else if(c == 0xAA && g_uart2Buffer[(j+6)&UART2_BUFFER_MASK] == 0xFF)
		{
			UART2_Config_B2("at+outtype=8\r\n");//为什么这么多设定模块传输格式的code？？？？
		}
	}
	return false;
}

bool dust_receiveData(int32_t retries)//多次尝试已确定接收到数据
{
	while(retries--)
	{
		if(get_dust_data()==true)
		{
			return true ;
		}
		HAL_Delay(1);
	}
	return false;
}
//#if 1
void dust_update(void)
{
	
		HAL_Delay(1);
		UART2_Config_B2("at+fco\r\n");
		HAL_Delay(2);
		UART2_Config_B2("at+outtype=8\r\n");
		HAL_Delay(2);
		dust_receiveData(5);
		memset(g_uart2Buffer,UINT8_MAX-1,sizeof(g_uart2Buffer));
}
//#endif
//#if 0
//void dust_update(void)
//{
//	{
//		if (!HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_5) && g_main_status!=STATUS_POWEROFF)
//		{
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_RESET);//B2 power off
//			
//			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_5,GPIO_PIN_SET);//B2 power on
//		}
//	
//		__IO uint32_t tbuffer[UART2_BUFFER_SIZE] = {0};
//		__IO uint8_t start = 0;
//		for(uint16_t i=0; i<UART2_BUFFER_SIZE; i++)
//		{
//			uint16_t c = g_uart2Buffer[i];
//			 if (c == 0x5a && g_uart2Buffer[(i+1) & UART2_BUFFER_MASK] == 0xa5 && g_uart2Buffer[(i+2) & UART2_BUFFER_MASK] == 0x34)
//			{
//				start = i;
//				
//				__IO uint16_t check = 0;
//				//buffer[0] = 0;
//				for( uint16_t j = 0; j<UART2_BUFFER_SIZE; j++)
//				{
//					tbuffer[j] = g_uart2Buffer[j+start & UART2_BUFFER_MASK];
////					if (j<=57)
////					{
////					check += tbuffer[j] ;
////					}
//					
//				}
//				check = 0;
//				for( uint16_t j = 0; j<56; j=j+4)
//				{				
//				//if (j<=57)
//					{
//					check += tbuffer[j] + tbuffer[j+1] + (tbuffer[j+2]<<8) +(tbuffer[j+3]<<16) ;
//					}
//				
//				}
//				if (((tbuffer[58]<<8)+(tbuffer[59]))==check)
//				{
//					uint32_t number1,number2_5,number10;
//					number1 = fmax(0,(tbuffer[11]<<24) +(tbuffer[10]<<16) +(tbuffer[9] <<8) + (tbuffer[8]));
//					number2_5 =fmax(0,(tbuffer[19]<<24) +(tbuffer[18]<<16) +(tbuffer[17]<<8) + (tbuffer[16]));
//					number10 = fmax(0,(tbuffer[35]<<24) +(tbuffer[34]<<16) +(tbuffer[33]<<8) + (tbuffer[32]));

////					if (number2_5 >= 200000 ) 
////					{
////						UART_config_B2("at+fco\r\n");
////					}

//					{
//						g_firPM.PM2D5V = 	fmin(99999,number1);
////						g_main_PM2_5Value = fmin(99999,number2_5);
////						g_main_PM10Value 	= fmin(99999,number10);
//					}
//				}

//	
//				return;
//			}
//			else if (c == 0xAA && g_uart2Buffer[(i+6) & UART2_BUFFER_MASK] == 0xFF)
//			{
//				
//				char str[15] = "at+outtype=4\r\n";
//				//UART_config_B2(str);	
//			
//			}
//		}
//	}
//}
//#endif

