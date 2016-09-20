
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdlib.h"
#include "esp8266.h"
#include "stm32f0xx_hal.h"

//hardware start here...
UART_HandleTypeDef* volatile g_esp8266_huart = NULL;
volatile uint32_t* volatile g_esp8266_dmaRemain = NULL;

//overwrite fputc function, or you will unable to use printf(...) function.
int fputc(int ch,FILE *f)
{
	if (g_esp8266_huart != NULL)
	{
		uint8_t c=ch;
		//PLATFORM uart transmit a single char.
		HAL_UART_Transmit(g_esp8266_huart,&c,1,1000);
	}
	return ch;
}

// this function return a (uint32_t) tick which should increase by 1 every 1ms.
static inline uint32_t getTick(void)
{
	//PLATFORM interrupt and volatile must be used.
	return HAL_GetTick();
}

// delay 1 ms
static inline void hw_delay(uint32_t ms)
{
	//PLATFORM
	HAL_Delay(ms);
}

// size depends on the RAM size. must be 1<<n. if you need more buffer for http response. declare a bigger one.
#define ESP8266_BUFFER_SIZE (1<<8)  //128  / 1000 0000
#define ESP8266_BUFFER_MASK (ESP8266_BUFFER_SIZE-1)//127 /  0111 1111��������������ĳ������������������۴����仯�ķ�Χ��ʹ����������0-127��Χ�ڱ仯�����涨�����ķ�Χ������0-127֮���ֻأ�

static void softwareInit(void);
//PLATFORM must be volatile, as it can be modified by DMA and IT.
volatile uint16_t g_esp8266_bufferIndex = 0;
volatile uint8_t g_esp8266_buffer[ESP8266_BUFFER_SIZE] = {0};

//pull down reset pin of the WIFI chip, then pull up.
static void hw_reset(void)
{
	//HARDWARE set the gpio below as your REST pin of WIFI chip.(would be gpio16 instead, see the sch of WIFI moudle)
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	hw_delay(10);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
}

static void hw_init(void)
{
	//PLATFORM 
	//Setp 0: power on
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
	//HARDWARE set the gpio below as your REST pin of WIFI chip.
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	//HARDWARE set the gpio below as your power MOSFET control pole if you have one. otherwise ignore.
  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
//  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	
	//////*over write  IGNORED *//////
	//Step 1: init the buffer to UINT16_MAX, so that we would know the buffer has been wright.
	//memset((void*)g_esp8266_buffer,UINT16_MAX,sizeof(g_esp8266_buffer));
	g_esp8266_dmaRemain = &(g_esp8266_huart->hdmarx->Instance->CNDTR);//CNDTR�Ĵ����м�Ϊ���յ������ݵ�����
																																		//��g_esp8266_dmaRemainΪһ��ָ�룬ָ����յ����ݵ�����
	//Step 2: Start DMA. In circle mode, dma would NEVER STOPPED.			//g_esp8266_dmaRemainΪָ���յ����ݵ�ָ��
	//start an automatic thread to collect the byte comming from RX pin.
	
	//////*over write  IGNORED *//////
	//the element of the buffer is 16bit, and has been set by 0xffff. 
	//if new byte comes, the element should be 0x00??, otherwise, it should be 0xff??. easy to find where to read.
	//as soon as the byte has be get by other function, please remember to set it by 0xffff. (this has already been done by the function below.)
	HAL_UART_Receive_DMA(g_esp8266_huart, (uint8_t *)g_esp8266_buffer, ESP8266_BUFFER_SIZE);
}

static void hw_deinit(void)
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_SET);
}


//PLATFORM   init with a uart port to communicate with WIFI chip. 
//the uart must has a dma handle, which from Rx to memory, P-increase disable, M-increase enable, circle mode, byte to byte.
//the dma interrupt must disable(which can be modified in uart.c), or the dma would be stop after the first circle.
void esp8266_init(UART_HandleTypeDef* huart)
{
	g_esp8266_huart = huart;
	hw_init();//��ʼ��Ӳ��������MCU��wifi���յ�������//Ӳ����أ�������أ���ʼ��
	softwareInit();
}

void esp8266_deinit(void)
{
	hw_deinit();
}

static inline uint16_t getWriteIndex(void)//�����鿴UART_DMA�������ݽ��յļĴ���ʣ��Ŀռ䣬ע�⣺������g_esp8266_dmaRemain�����յ����ݵ����������������ݽ�β���ġ�\0�������ַ�
{
	return ESP8266_BUFFER_SIZE - *(g_esp8266_dmaRemain);
}

/******************************************************************************/
/***********************************WARNING************************************/
/******************************************************************************/
/********software start... beginner would NOT change any code below.***********/
/********if you need call some function, please read ESP8266.h before**********/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

__IO uint8_t g_esp8266_status = 5;
uint32_t g_esp8266_smartTick  = 0;
uint32_t g_esp8266_pairTick = 0;
uint8_t g_esp8266_pairTodo = 0;
uint8_t g_esp8266_isAtMode = 0;
uint32_t g_esp8266_resetTick = 0;
uint8_t g_esp8266_smartStarted = 0;
char* g_esp8266_receiveString = NULL;
char* g_esp8266_ssidString = NULL;
char* g_esp8266_passwordString = NULL;
void(*g_esp8266_smartFirstReceived)(void) = NULL; 
void(*g_esp8266_smartSsidAndPasswordReceived)(char*, char*) = NULL; 

void esp8266_setSsidAndPasswordCallback(void(*fcn)(char*, char*))
{
	g_esp8266_smartSsidAndPasswordReceived = fcn;
}

void esp8266_setSmartCallback(void(*fcn)(void))//ע����ֻ��һ������Ϊ��void���ͺ������ĺ��������þ�����ִ�в���������ִ����������������������
{
	g_esp8266_smartFirstReceived = fcn;
}
cJSON * g_esp8266_responseJson = NULL;

uint8_t jsonResponse(char* str)
{
	if(*str == '{')
	{
		cJSON* json = cJSON_Parse(str);
		if (json)
		{
			if (g_esp8266_responseJson)
			{
				cJSON_Delete(g_esp8266_responseJson);
			}
			g_esp8266_responseJson = json;
			return 1;
		}
	}
	return 0;
}

uint8_t cipClosed(char* str)
{
	if(strcasecmp(str,"CLOSED")==0)
	{
		//reset();
		g_esp8266_status = 2;
		return 1;
	}
	
	if(strcasecmp(str,"link is not valid")==0)
	{
		//reset();
		g_esp8266_status = 2;
		return 1;
	}
	
	return 0;
}

uint8_t smartInfo(char* str)//���� �Ѿ���readNewLine()������Ľ������� ��1.������wifi�ɹ��󣬹ر�smart���̣�2����
{
	if (g_esp8266_smartStarted)//g_esp8266_smartStartedΪWIFI���ñ�־λ
	{
		if(strstr(str,"smartconfig connected wifi"))
		{
			//g_esp8266_smartFirstReceived();//��STM32�У�������ָ�벻�ּң�����Ҳ������Ϊһ��ָ�룬��esp8266_setSmartCallback(void(*fcn)(void))�����У�
																			//ֻ��һ��������g_esp8266_smartFirstReceived = fcn����esp8266_setSmartCallback������Ϊ�˺�������ĵ��ã�
																			//һ������������wifiSmartLinkStarted�ģ���������pairStart��������������pair()���ã���ʡȥ
			esp8266_smartStop();//ֹͣsmart����
//			if (g_esp8266_ssidString && g_esp8266_passwordString)
//			{
//				if (g_esp8266_smartSsidAndPasswordReceived)
//				{
//					g_esp8266_smartSsidAndPasswordReceived(g_esp8266_ssidString,g_esp8266_passwordString);//wifiSmartLinkStarted(g_esp8266_ssidString,g_esp8266_passwordString)
//				}
//			}
			return 1;
		}
		if(strstr(str,"smartconfig type:"))
		{
			if (g_esp8266_smartFirstReceived)
			{
				g_esp8266_smartFirstReceived();//����=pairStart����
			}
			if (g_esp8266_smartTick - (getTick() + 60000) > (UINT32_MAX>>1) )//ע�⣺�����ǡ�>��(UINT32_MAX>>1),���Ա�ʾ"<0"������ʵ����ʲô��ˢ��g_esp8266_smartTick�𣿣�
			{
				g_esp8266_smartTick = getTick() + 60000;
			}
			return 1;
		}
		if(strstr(str,"ssid:"))
		{
			char* ssid = str+5;//������ʲô������
			if (strlen(ssid)>0)
			{
				if (g_esp8266_ssidString)
				{
					free(g_esp8266_ssidString);
				}
					g_esp8266_ssidString = (char*) malloc(strlen(ssid)+1);
				if (g_esp8266_ssidString)
				{
					strcpy(g_esp8266_ssidString,ssid);
				}
			}
		}
		if(strstr(str,"password"))
		{
			char* password = str+9;
			if (strlen(password)>0)
			{
				if (g_esp8266_passwordString)
				{
					free(g_esp8266_passwordString);
				}
					g_esp8266_passwordString = (char*) malloc(strlen(password)+1);
				if (g_esp8266_passwordString)
				{
					strcpy(g_esp8266_passwordString,password);
				}
			}
			
		}
	}
	return 0;
}
uint8_t statusChanged(char* str)
{
	uint8_t status = 0;
	if(strcmp(str,"WIFI DISCONNECT") == 0)
	{
		status = 5;
	}
	if(strcmp(str,"WIFI CONNECTED") == 0)
	{
		//status = 4;
	}
	if(strcmp(str,"WIFI GOT IP") == 0)
	{
		status = 2;
	}
	if(strstr(str,"STATUS:") && strlen(str) == 8)
	{
		status = str[7] - '0';
	}
	
	if(strcasecmp(str,"CONNECT")==0)
	{
		status = 3;
	}
	if(strcasecmp(str,"CLOSED")==0)
	{
		status = 2;
	}
	
	if(strcasecmp(str,"link is not valid")==0)
	{
		status = 2;
	}	
	if(strcasecmp(str,"ALREADY CONNECTED")==0)
	{
		status = 3;
	}
	if(strstr(str,"+CIPSTATUS:0,\"UDP\","))
	{
//		status = 3;
	}
	if (status != 0)
	{
		g_esp8266_status = status;
		return 1;
	}
	return 0;
}

uint8_t receiveData(char* str)
{
	if(strstr(str,"+IPD,"))
	{
		char* douhao = strchr(str,',');
		char* maohao = strchr(str,':');
		if (douhao && maohao)
		{
			*douhao = 0;
			*maohao = 0;
			uint8_t len = atoi(douhao+1);
			if (g_esp8266_receiveString)
			{
				free(g_esp8266_receiveString);
			}
			g_esp8266_receiveString = malloc(len+1);
			strncpy(g_esp8266_receiveString,maohao+1,len);
			//printf("RECV:%d,%s",len,maohao+1);
			return 1;
		}
	}
	return 0;
}
typedef uint8_t(*focus_t)(char*);//ʹ��typedef����ĺ���ָ�룬�ܹ��������Ӧ�ã�����ָ�����ַ�ʽ��uint8_t(*focus_t)(char*)��typedef uint8_t(*focus_t)(char*)��ǰһ��Ӧ��ʱ
																 //��Ҫ�Ѻ���������focus_t=g_esp8266_focus������Ӧ�ø�focus_t������һ��Ӧ��ʱ�൱�ڶ�����һ��������ģ�壬ֱ��focus_t g_esp8266_focus��Ӧ��g_esp8266_focusӦ����
focus_t g_esp8266_focus[] = {smartInfo,statusChanged,receiveData};//����һ���������飬��һ��������������Ԫ�ص�����
																																	//smartInfo�����������ݣ�




//smartlink״̬�У�wifi״̬�У�ץȡ������//��һ�£��������á�������
																						//��Ϊ�β�����HAL�������DMA���շ�ʽ�أ������Ѿ���DMA��ʽ���յ������ݣ������ǽ������ݵĴ���
static uint16_t checkNewLine(void)//MCU��wifiģ��������ݣ�DMA��ʽ������DMA��ʽ������ѭ���޼���ģ�����һ�εĴ���������κ�λ�ÿ�ʼ�����Խ��շ�ʽ���ܻ�Ƚ�����
{	//����������ص��Ǳ��δ������ݵĸ��������ܵ�һ�β���ʹ���ݴ�����һ�����ݵĵ�һ������ʼ���䣬�������ڴ����ݵĽ�β���������ٴδ��������ʹDMA�������ݵ�һλ��buffer�еĵ�һλ�����ˣ����ԣ��˺������ص���һ�����ݵĸ���
	for (uint16_t i = 0; i < ESP8266_BUFFER_SIZE; i++) //#define SIZE_FOR_UART_RX 16 //i�� 0 �� 127�仯
	{
		uint16_t index = (g_esp8266_bufferIndex+i)&ESP8266_BUFFER_MASK;//����DMA���䷽ʽ��ѭ���޼�������ض����������ݣ�ǰ��Ҳ˵��һ�δ�������Ǵ��κ�λ�ÿ�ʼ��Ϊ�˷�ֹ���ܵ�ʱ��
												//�����ݷŴ�λ�ã������趨��g_esp8266_bufferIndex������ʾ�����ݵ���ʼλ�ã��������������readBuffer����ȷ���ģ���i��Զ�Ǵ�1��127�仯�ģ�����g_esp8266_bufferIndex
												//�������������£�g_esp8266_bufferIndex+i>127������MASK=0111 1111������indexֵ���ٴν���0-127��ѭ��������������Ū����ô�鷳����Ϊ�������۴������ݵ���ʼλ�ã���������ʱ��������ȷ��ѭ���У�
		if (index == getWriteIndex())//��Index==getWriteIndex()=ESP8266_BUFFER_SIZE - *(g_esp8266_dmaRemain),g_esp8266_bufferIndex=getWriteIndex�����Ե�index=g_esp8266_bufferIndex
		{														 //ʱ������ѭ����������Ϊ�Σ���
			break;
		}
		uint16_t c = g_esp8266_buffer[index];
		if (c == 0x0a || c == 0x00)
		{
			return i+1;   //�����������ַ�ʱ�����ݽ�����ᣬֱ�ӽ����ݽ��ռ���i��1����������ٽ���forѭ��
		}
	}
	return 0;
}

static uint16_t readBuffer(char * data, uint16_t maxlen)//��maxlen���ȵĵ������յ������ݷŵ�����data�����У�������current_lenҲ�����ݳ���=maxlen
{
	uint16_t current_len = 0;
	for (uint16_t i = 0; i < maxlen; i++) //#define SIZE_FOR_UART_RX 16
	{
		if (g_esp8266_bufferIndex == getWriteIndex())//????���bufferIndex�������ϴ������ݸ�������buffer����ʱ����ô�ˣ�����ʵ��
		{
			break;
		}
		uint16_t c = g_esp8266_buffer[g_esp8266_bufferIndex];
		g_esp8266_bufferIndex = (g_esp8266_bufferIndex + 1)&ESP8266_BUFFER_MASK;
		*(data+current_len) = c;
		current_len++;
	}
	return current_len;
}

static char* readNewLine(void)//�������յ������ݽ�����Ӧ����������ĩλ�͵����ڶ�λ��Ϊ��/0��������return�洢������ݵ�bufffer
{
	uint16_t lineLength = checkNewLine();//���ر��ν��յ����ݵĸ���
	if (lineLength == 0)
	{
		return NULL;
	}
	char* buffer = malloc(lineLength);
	if (buffer)
	{
		readBuffer(buffer,lineLength);//�ѽ��յ���lineLength�������ݴ��͵�buffer�У����������ݳ��ȡ�
		*(buffer+lineLength-1) = '\0';//����malloc����ռ�����һλΪ  \0(C���Ե��ַ���������)
		if (lineLength>1)
		{
			char c = *(buffer+lineLength-2);//��������buffer�е������еĵ����ڶ�λ�洢�����ݣ��س����߻��з�����Ϊ����λ����Ϊ"\0"��
			if (c == 0x0d || c == 0x0a)
			{
				*(buffer+lineLength-2) = '\0';
			}
		}
	}
	return buffer;
}

static inline uint16_t lastIndex(void)//�򱾺����������Ƿ���{���ݽ��ռĴ���}�����һλ����{���ݰ�}�е�λ�ã�ע�⣺�������ݽ��ո��������������ַ���\0��,����Ҫ��ȥ1���ַ���
{
	return (getWriteIndex()-1)&ESP8266_BUFFER_MASK;
}
static char bufferLastChar(void)//����{���ݽ��ռĴ���}�е����һλ���ݣ���{���ݽ��ռĴ���}��ʣ��ռ����g_esp8266_bufferIndex�����һλ���ݼ�Ϊ��\0��
{
	if (g_esp8266_bufferIndex == getWriteIndex())// getWriteIndex()�������ݽ��ռĴ�����ʣ��ռ䣬Ϊ��g_esp8266_bufferIndex�������������ݽ�β����Щ�������б����𣿣���
	{
		return '\0';
	}
	else
	{
		return g_esp8266_buffer[lastIndex()];
	}
}



void readFocus(void)
{
	char* line = NULL;
	while(1)
	{
		line = readNewLine();
		if (line)
		{
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))//line����readFocus�����������Ĳ���
				{
					break;
				}
			}
			free(line);
		}
		else
		{
			break;
		}
	}
}

uint32_t waitUntilBlankLine(uint32_t ms)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strlen(line) == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}


uint32_t waitUntilRxStopped(uint32_t ms, uint16_t interval)//������˵��ms��wait�����ʱ�䣬interval������ģ�鷵�ؼ�������ʱ�䣻��MCU���msʱ��û�н�������ʱ������0��
													//��û�еȹ�msʱ���ڣ������ν������ݼ��ʱ�����intevalʱ�����أ�����ֵΪʣ��ɵȴ�ʱ��timeOutTick - currentTick
{
	uint32_t timeOutTick = getTick() + ms;//ֻ����һ�Σ��������к�Ϊ��ֵ������������������һ��������getTick�йأ�����Щ�����ǲ��ϸ��µģ�����currentTick = getTick();
	uint32_t lastRxTick = getTick();
	uint32_t currentTick = getTick();
	uint16_t i = lastIndex(),j;
	while(timeOutTick - currentTick< (UINT32_MAX>>1))//�൱��currentTickһֱ�ڸ��£�ֱ�����µ�timeOutTickС��currentTickΪֹ
	{
		j = i;
		i = lastIndex();//lastIndex()=��ESP8266_BUFFER_SIZE-*(g_esp8266_dmaRemain)-1��& ESP8266_BUFFER_MASK;*(g_esp8266_dmaRemain)Ϊ���յ����ݵ�������
										//i����һ���̶��ϱ������ݽ�����������������Ϊ�ο���ô��ʾ�����󣿣���
		if(i!=j)//����д���Ľ��գ���buffer��Ϊ�������ݵĿռ䷢���仯����i!=j,�����lastRxTick
		{
			lastRxTick = getTick();
		}
		if (currentTick - (lastRxTick + interval) < (UINT32_MAX>>1))//����currentTickһֱ���£������ϴν������ݵ�ʱ��Ҳ��һֱ���µģ�ֻ�е��ϴν�������ʱ�䲻�ٸ��£�
																																//Ҳ���ǲ��ٽ�������ʱ�����������Ч�����ԣ���code��˼�����δ�����ʱ�����intervalʱ�����صȴ�ʱ���ʣ��
		{
			return timeOutTick - currentTick;
		}
		currentTick = getTick();//�����Ƕ�currentTick�ĸ���
	}
	return 0;
}

uint8_t waitUntilOk(uint32_t ms)//getTick�ǰ����ڻ�ȡ��ʱ��ʱ��      //�˺���������ȷ��ָ��ͳɹ������ɹ��򷵻ط���ֵ�������ɹ����򷵻���ֵ
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strcmp(line,"OK") == 0 || strcmp(line,"SEND OK") == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			if (strcmp(line,"ERROR") == 0 || strcmp(line,"busy s...") == 0)
			{
				free(line);
				return 0;
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)//ֱ��д��3�����𣿣���
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
uint8_t waitUntilStrcmp(uint32_t ms, char* str)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strcmp(line,str) == 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
	
uint8_t waitUntilStrstr(uint32_t ms, char* str)
{
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if (strstr(line,str) != 0)
			{
				free(line);
				return timeOutTick - currentTick;
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 0;
}
static void vSetCommand(char * commond,va_list argptr)
{
	char* connect = "=";
	char * para;
	if (commond!=NULL)
	{
		printf("AT+%s",commond);
		while ((para = va_arg(argptr,char*))!=NULL)
		{
			printf("%s%s",connect,para);
			connect = ",";
		}
	}
	else
	{
		printf("AT");		
	}
	printf("\r\n");
}
static void setCommand(char* commond, ...)
{
	va_list argptr;
	va_start(argptr, commond);
	vSetCommand(commond,argptr);
	va_end(argptr);
}

static void setCommandWait(uint32_t ms, char * commond,...)
{
	waitUntilRxStopped(10,2);
	readFocus();
	va_list argptr;
	va_start(argptr, commond);
	vSetCommand(commond,argptr);
	va_end(argptr);
	waitUntilRxStopped(waitUntilBlankLine(ms),25);
}
char* getCommand(uint32_t ms, char* commond)
{
	printf("AT+%s\r\n",commond);
	uint32_t timeOutTick = getTick() + ms;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if(line[0] == '+')
			{
				uint16_t i;
				for(i = 1; i<strlen(line);i++)
				{
					if(line[i] == ':' || line[i] == '=')
					{
						break;
					}
				}
				uint16_t size = strlen(line) - i;
				if (size)
				{
					char* rst = malloc(size);
					if (rst)
					{
						strcpy(rst,line+i+1);
					}
					free(line);
					return rst;
				}
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return NULL;
}

static void setATMode(void)
{
	hw_delay(500);
	readFocus();
	printf("+++");
	hw_delay(500);
	printf("\n");
	setCommandWait(5000,NULL);
	setCommandWait(5000,NULL);
	g_esp8266_isAtMode = 1;
}

static void reset(void)//���ģ������ǰ��������wifi����wifiģ��Ӳ��������ʹģ�����wifi�����ã�������ǰû������wifi����ʲô����ִ��
{
	hw_reset();//ע��g_esp8266_smartStarted��wifi���õı�־λ����ʾ�ǵ��ǵ�wifi����ʱ��־λΪ1�������Ǳ�ʾ����־λΪ1ʱ����wifi���ã����Ծ���ģ��Ӳ���ã�����־λ�Ծ�Ϊ1��ֻ��
						 //wifi���õĺ����Ѿ��ж���
	if (g_esp8266_isAtMode)//g_esp8266_isAtMode��ʲô��־������
	{
	//	setATMode();
	}
	if (g_esp8266_smartStarted)
	{
		uint32_t smartRemain = g_esp8266_smartTick - getTick();//ע���������hw_reset����ֻ��wifiģ���Ӳ�����������ᵼ��MCU��ʧ���ݣ���������ǰ��wifi���ú���smart�����е�
																													 //����ʱ��ms���ɴ��ڣ�Ҳ�������ڼ������ smartRemain = g_esp8266_smartTick - getTick()�����Խ������ٴ�����smart����ʱ������esp8266_smart(smartRemain)����
		if (smartRemain<(UINT32_MAX>>1) && smartRemain>0)
		{
			esp8266_smart(smartRemain);//
		}
	}
	g_esp8266_resetTick = getTick();
}

uint8_t getStatus(uint8_t force)
{
	if (!force)
	{
		if (g_esp8266_status == 3)
		{
			return g_esp8266_status;
		}
	}
	setCommand("CIPSTATUS",NULL);
	uint32_t timeOutTick = getTick() + 1000;
	uint32_t currentTick = getTick();
	while(timeOutTick - currentTick< (UINT32_MAX>>1))
	{
		char* line = readNewLine();
		if (line)
		{
			if(strstr(line,"STATUS:") && strlen(line) == 8)
			{
				g_esp8266_status = line[7] - '0';
				free(line);
				return g_esp8266_status;
			}
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)
			{
				if (g_esp8266_focus[i](line))
				{
					break;
				}
			}
			free(line);
		}
		currentTick = getTick();
	}
	return 5;
	
}
void esp8266_run(void)
{
	readFocus();//?????what's the meaning?
	if(g_esp8266_smartStarted)//Ҳ����һֱ�ڼ��smart�����Ƿ�ʱ������ʱ��ֹͣsmart����ˢ��g_esp8266_resetTick = getTickʱ��
	{
		if ((getTick() - g_esp8266_smartTick)<(UINT32_MAX>>1))//g_esp8266_smartTick=getTick+ms,�ӿ�ʼ���Ͳ���ˢ�£����ԣ������ʾ����smartlink���̳�ʱ
		{
			esp8266_smartStop();//ע�⣬�˾����ֹͣsmart�����Ժ󣬻��g_esp8266_smartStarted��0
			reset();//����esp8266_smartStop()���g_esp8266_smartStarted��0���������codeֻ��ˢ��g_esp8266_resetTick��ʱ��
		}
	}
	//esp8266_resetForCrash(60000);
}

void esp8266_sendHTTPRequest(char* path, cJSON* json)
{
	if (g_esp8266_isAtMode) 
	{
		return;
	}
	char * str = cJSON_PrintUnformatted(json);
	if (str)
	{
		uint16_t length = strlen(str);
		char* head = "POST %s HTTP/1.1\r\n"
		"Content-Length: %i\r\n"
		"Content-Type: text/plain\r\n"
		"Host: api.hanwanglantian.com\r\n"		
		"Connection: keep-alive\r\n"
		"User-Agent: konggan\r\n"
		"Accept: \r\n"		
		"\r\n";
		printf(head,path,length);
		printf(str);
//	printf("\r\n");
		free(str);
	}
}

cJSON* esp8266_getHTTPResponse(void)
{
	if (g_esp8266_responseJson)
	{
		cJSON* json = g_esp8266_responseJson;
		g_esp8266_responseJson = NULL;
		return json;
	}
	else
	{
		return NULL;
	}
}

static void softwareInit(void)
{
	//Step 3: reset
	reset();//���ģ������ǰ��������wifi����wifiģ��Ӳ��������ʹģ�����wifi�����ã�������ǰû������wifi����ʲô����ִ��
	//Step 4: wait until the "ready" appeared.
	hw_delay(500);
	waitUntilRxStopped(5000,100);//MCU�ڵȴ�ʱ��ms��û�н�������ʱ������0����û�еȹ�msʱ���ڣ������ν������ݼ��ʱ�����intevalʱ������ʣ��ɵȴ�ʱ��timeOutTick - currentTick
	readFocus();//?????�������յ�������
	//Step 5: check if in TM mode.
	printf("\n");
	hw_delay(1000);
	uint8_t test = lastIndex();//???
	char c = bufferLastChar();//????
	if (c != '\n' )
	{
		//Setp 5.1: if not in TM mode. Set the default properties.
		setATMode();//���Ǻ��⣿��
	}
	setCommandWait(5000,"CWMODE_DEF","1",NULL);  //sta mode.   ��˼�ǣ�at+CWMODE_DEF=1
	//*
//	char* ssid = "\"AIRJ_NB\"";
//	setCommandWait(20000,"CWJAP_DEF?",NULL);
//	setCommandWait(20000,"CWJAP_DEF",ssid,"\"airj2123\"",NULL); //default ssid.
	//*/
	setCommandWait(5000,"CWAUTOCONN","1",NULL);  //auto connect after reset.
//		setCommandWait(5000,"CIPSTART","\"TCP\"",g_esp8266_tcpServerIP,g_esp8266_tcpServerPort,NULL);  //set tcp target
	setCommandWait(5000,"CIPMODE","0",NULL);  //
	setCommandWait(5000,"SAVETRANSLINK","0",NULL);  //
	setCommandWait(5000,"CIPMUX","0",NULL);  //
	g_esp8266_isAtMode = 0;//�Ƿ���͸��ģʽ�£��Ѳ��ã��Լ���������������
//		reset();  // reset to enter tm mode.
//		waitUntilRxStopped(waitUntilBlankLine(5000),25);
//		readFocus();
}
uint8_t smart(int8_t type)//wifi���ú���
{
	if (type == 0)
	{
		setCommandWait(0,"CWSTARTSMART",NULL);
	}
	else
	{
		char buffer[4];
		sprintf(buffer,"%d",type);
		setCommandWait(0,"CWSTARTSMART",buffer,NULL);
	}
	if (waitUntilOk(5000))
	{
		return 1;
	}
	else
	{
		type--;
		if(type == 4 || type == -1)
		{
			return 0;
		}
		else
		{
			return smart(type);//����������Ƕ�ף���type=0,1,2,3ʱ��ѭ��ִ�б�����,���waitUntilOk�з��أ���return1������޷��أ����3��1��ʽ��ִ��һ��
		}
	}
}
void esp8266_smart(uint32_t ms)//����wifi����smart(3)�����ѱ�־λg_esp8266_smartStarted��1���趨����ʱ��ms
{
	if(smart(3))//����3��ʽͨѶ����wifi��ESP8266ָ��pdf����û����Ӧ��ָ������Ϊ���pdf�汾̫��
	{
		g_esp8266_smartStarted = 1;//wifi���õı�־λ
		g_esp8266_smartTick = getTick()+ms;//
		
	}
	readFocus();
}

void esp8266_smartStop(void)//ֹͣsmart���̣�����ָֹͣ����ҵȴ�5s�����Ƿ��ͳɹ���
{
	setCommandWait(5000,"CWSTOPSMART",NULL);
	if (waitUntilOk(5000))
	{
		g_esp8266_smartStarted = 0;
	}
}


uint8_t esp8266_smarting(void)
{
	return g_esp8266_smartStarted;
}
uint8_t esp8266_getStatus(void)
{
	if (g_esp8266_smartStarted)
	{
		return 1;
	}
	if (g_esp8266_status == 2)
	{
		return 4;
	}
	return g_esp8266_status;
}
/*
void esp8266_connectTCP(char* address, uint16_t port)
{
	char buffer[0x10];
	sprintf(buffer,"%d",port);
	char* addressForAT = (char*)malloc(strlen(address)+3);
	sprintf(addressForAT,"\"%s\"",address);
	setCommandWait(5000,"CIPSTART","\"TCP\"",address,buffer,NULL);
	free(addressForAT);
}

uint8_t esp8266_tcp(char * str)
{
//	char* status = getCommand(1000,"CIPSTATUS");
	if (!g_esp8266_smartStarted)
	{
		uint8_t status = getStatus(0);
		if(status == 2 || status == 4 || status == 3)
		{
			esp8266_connectTCP("192.168.2.89",82);
		}
		else if (status == 3)
		{
			char buffer[0x10];
			sprintf(buffer,"%d",strlen(str));
			setCommandWait(0,"CIPSENDEX",buffer,NULL);
			if(waitUntilOk(500))
			{
				printf(str);
				waitUntilOk(5000);
			}
		}
	}
	return esp8266_getStatus();
}*/


void esp8266_connectUDP(char* address, uint16_t port)
{
	char buffer[0x10];
	sprintf(buffer,"%d",port);
	char* addressForAT = (char*)malloc(strlen(address)+3);
	sprintf(addressForAT,"\"%s\"",address);
	setCommandWait(5000,"CIPSTART","\"UDP\"",addressForAT,buffer,NULL);
	free(addressForAT);
}
uint8_t esp8266_udp(char * str, char* address, uint16_t port)
{
	static char* lastAddress = NULL;
	static uint16_t lastPort = 0;
	if (!g_esp8266_smartStarted)
	{
		uint8_t status;
		if (port!=lastPort || strncmp(address,lastAddress,strlen(address)))
		{
			esp8266_disconncet(0);
			status = g_esp8266_status;
		}
		else
		{
			status = getStatus(0);
		}
		if(status == 2 || status == 4)
		{
			esp8266_connectUDP(address,port);
			lastAddress = address;
			lastPort = port;
		}
		else if (status == 3)
		{
			char buffer[0x10];
			sprintf(buffer,"%d",strlen(str));
			setCommandWait(0,"CIPSENDEX",buffer,NULL);
			if(waitUntilOk(500))
			{
				printf(str);
				waitUntilOk(5000);
			}
		}
	}
	return esp8266_getStatus();
}

uint8_t esp8266_disconncet(uint8_t force)
{
	if (!g_esp8266_smartStarted)
	{
		uint8_t status = getStatus(0);
		if(force || status == 3)
		{
			setCommandWait(0,"CIPCLOSE",NULL);
			if(waitUntilOk(500))
			{
				return 1;
			}
		}
	}
	return 0;
}

void esp8266_reset(void)
{
	reset();
}
char* esp8266_getSSID(void)
{
	if (g_esp8266_ssidString == NULL)
	{
		if(!g_esp8266_smartStarted)
		{
			char* get = getCommand(200,"CWJAP_DEF?");
			if(get)
			{
				char* start,*end;
				start = strstr(get,"\"");
				end = strstr(get,"\",\"");
				if(start && end && start+1<end)
				{
					char* ssid = malloc(end-start);
					*end = '\0';
					strcpy(ssid,start+1);
					g_esp8266_ssidString = ssid;
				}
				free(get);
			}
		}
	}
	if (g_esp8266_ssidString)
	{
		char* ssid = malloc(strlen(g_esp8266_ssidString)+1);
		strcpy(ssid,g_esp8266_ssidString);
		return ssid;
	}
	return NULL;
}
char* esp8266_getReceiveString(void)
{
	char* str = g_esp8266_receiveString;
	g_esp8266_receiveString = NULL;
	return str;
}
void esp8266_test(void)
{
	setCommandWait(1000,"GMR",NULL);
//	char* ssid = "\"AIRJ_NB\"";
//	setCommandWait(20000,"CWJAP_DEF",ssid,"\"airj2123\"",NULL); //default ssid.
//	setCommandWait(0,"CIUPDATE",NULL);
//	if(waitUntilStrcmp(5000,"+CIPUPDATE:4"))
//	{
//		waitUntilStrstr(60000,"jump to run user");
//	}
	
//	setCommandWait(0,"CWSAP","\"AIRJESPAIRJ\"","\"airj2123\"","11","3",NULL);
}

uint8_t esp8266_setSsidAndPassword(char *ssid, char *password)
{
	char rssid[34], rpassword[34];
	sprintf(rssid,"\"%s\"",ssid);
	sprintf(rpassword,"\"%s\"",password);
	setCommandWait(1,"CWJAP_DEF",rssid,rpassword,NULL);
	return waitUntilOk(20000)>0;
}
