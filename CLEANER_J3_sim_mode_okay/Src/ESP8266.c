
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
#define ESP8266_BUFFER_MASK (ESP8266_BUFFER_SIZE-1)//127 /  0111 1111，它的作用是让某个数字与它想与后，无论此数变化的范围，使相与后的数在0-127范围内变化（即规定此数的范围，并在0-127之间轮回）

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
	g_esp8266_dmaRemain = &(g_esp8266_huart->hdmarx->Instance->CNDTR);//CNDTR寄存器中即为接收到的数据的数量
																																		//即g_esp8266_dmaRemain为一个指针，指向接收到数据的数量
	//Step 2: Start DMA. In circle mode, dma would NEVER STOPPED.			//g_esp8266_dmaRemain为指向收到数据的指针
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
	hw_init();//初始化硬件启动和MCU从wifi接收到的数据//硬件相关（串口相关）初始化
	softwareInit();
}

void esp8266_deinit(void)
{
	hw_deinit();
}

static inline uint16_t getWriteIndex(void)//用来查看UART_DMA用来数据接收的寄存器剩余的空间，注意：可能是g_esp8266_dmaRemain（接收到数据的数量）不包含数据结尾处的“\0”结束字符
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

void esp8266_setSmartCallback(void(*fcn)(void))//注意这只是一个参数为“void类型函数”的函数，作用就是先执行参数程序，再执行这个函数本身包含的内容
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

uint8_t smartInfo(char* str)//分析 已经被readNewLine()处理过的接收数据 ：1.在连接wifi成功后，关闭smart过程；2，在
{
	if (g_esp8266_smartStarted)//g_esp8266_smartStarted为WIFI配置标志位
	{
		if(strstr(str,"smartconfig connected wifi"))
		{
			//g_esp8266_smartFirstReceived();//在STM32中，函数与指针不分家，所以也可以视为一个指针，而esp8266_setSmartCallback(void(*fcn)(void))函数中，
																			//只有一个操作：g_esp8266_smartFirstReceived = fcn，而esp8266_setSmartCallback函数是为了函数方便的调用，
																			//一般是用来调用wifiSmartLinkStarted的，在其中有pairStart（）；在这里无pair()调用，可省去
			esp8266_smartStop();//停止smart过程
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
				g_esp8266_smartFirstReceived();//本句=pairStart（）
			}
			if (g_esp8266_smartTick - (getTick() + 60000) > (UINT32_MAX>>1) )//注意：这里是“>”(UINT32_MAX>>1),所以表示"<0"，这里实在做什么？刷新g_esp8266_smartTick吗？？
			{
				g_esp8266_smartTick = getTick() + 60000;
			}
			return 1;
		}
		if(strstr(str,"ssid:"))
		{
			char* ssid = str+5;//这里是什么？？？
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
typedef uint8_t(*focus_t)(char*);//使用typedef定义的函数指针，能够更方便的应用；函数指针两种方式：uint8_t(*focus_t)(char*)与typedef uint8_t(*focus_t)(char*)，前一个应用时
																 //需要把函数名赋给focus_t=g_esp8266_focus，依旧应用给focus_t，而后一个应用时相当于定义了一个函数的模板，直接focus_t g_esp8266_focus，应用g_esp8266_focus应即可
focus_t g_esp8266_focus[] = {smartInfo,statusChanged,receiveData};//这是一个函数数组，即一个包含三个函数元素的数组
																																	//smartInfo用来分析数据；




//smartlink状态中，wifi状态中，抓取的数据//试一下，看看作用·？？？
																						//那为何不能用HAL库给定的DMA接收方式呢？这里已经用DMA方式接收到了数据，这里是进行数据的处理
static uint16_t checkNewLine(void)//MCU从wifi模块接收数据，DMA方式；由于DMA方式传输是循环无间隔的，并且一次的传输可能是任何位置开始，所以接收方式可能会比较特殊
{	//正常情况返回的是本次传输数据的个数，可能第一次不会使数据从完整一次数据的第一个数开始传输，但随后会在此数据的结尾处结束，再次传输就正好使DMA传输数据第一位与buffer中的第一位对上了，所以，此函数返回的是一个数据的个数
	for (uint16_t i = 0; i < ESP8266_BUFFER_SIZE; i++) //#define SIZE_FOR_UART_RX 16 //i从 0 到 127变化
	{
		uint16_t index = (g_esp8266_bufferIndex+i)&ESP8266_BUFFER_MASK;//由于DMA传输方式是循环无间隔传输特定数量的数据，前面也说了一次传输可能是从任何位置开始，为了防止接受的时候
												//将数据放错位置，所以设定了g_esp8266_bufferIndex，它表示了数据的起始位置（它的是由下面的readBuffer（）确定的），i永远是从1到127变化的，在了g_esp8266_bufferIndex
												//不等于零的情况下，g_esp8266_bufferIndex+i>127，由于MASK=0111 1111，所以index值会再次进入0-127的循环。（所以这里弄得这么麻烦，是为了让无论传输数据的起始位置，接收数据时都处在正确的循环中）
		if (index == getWriteIndex())//即Index==getWriteIndex()=ESP8266_BUFFER_SIZE - *(g_esp8266_dmaRemain),g_esp8266_bufferIndex=getWriteIndex，所以当index=g_esp8266_bufferIndex
		{														 //时，跳出循环，这又是为何？？
			break;
		}
		uint16_t c = g_esp8266_buffer[index];
		if (c == 0x0a || c == 0x00)
		{
			return i+1;   //当有这两个字符时，数据接收完结，直接将数据接收计数i加1并输出，不再进入for循环
		}
	}
	return 0;
}

static uint16_t readBuffer(char * data, uint16_t maxlen)//将maxlen长度的的所接收到的数据放到数组data【】中，并返回current_len也是数据长度=maxlen
{
	uint16_t current_len = 0;
	for (uint16_t i = 0; i < maxlen; i++) //#define SIZE_FOR_UART_RX 16
	{
		if (g_esp8266_bufferIndex == getWriteIndex())//????这个bufferIndex参数加上传输数据个数等于buffer个数时就怎么了？？？实验
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

static char* readNewLine(void)//对所接收到的数据进行相应处理（将数据末位和倒数第二位置为“/0”），并return存储这个数据的bufffer
{
	uint16_t lineLength = checkNewLine();//返回本次接收到数据的个数
	if (lineLength == 0)
	{
		return NULL;
	}
	char* buffer = malloc(lineLength);
	if (buffer)
	{
		readBuffer(buffer,lineLength);//把接收到的lineLength长的数据传送到buffer中，并返回数据长度。
		*(buffer+lineLength-1) = '\0';//赋予malloc分配空间的最后一位为  \0(C语言的字符串结束符)
		if (lineLength>1)
		{
			char c = *(buffer+lineLength-2);//将所传到buffer中的数据中的倒数第二位存储的数据（回车或者换行符）置为结束位（即为"\0"）
			if (c == 0x0d || c == 0x0a)
			{
				*(buffer+lineLength-2) = '\0';
			}
		}
	}
	return buffer;
}

static inline uint16_t lastIndex(void)//则本函数的作用是返回{数据接收寄存器}的最后一位数在{数据包}中的位置；注意：由于数据接收个数不包括结束字符“\0”,所以要减去1个字符数
{
	return (getWriteIndex()-1)&ESP8266_BUFFER_MASK;
}
static char bufferLastChar(void)//返回{数据接收寄存器}中的最后一位数据；当{数据接收寄存器}的剩余空间等于g_esp8266_bufferIndex，最后一位数据即为“\0”
{
	if (g_esp8266_bufferIndex == getWriteIndex())// getWriteIndex()返回数据接收寄存器的剩余空间，为何g_esp8266_bufferIndex等于他就是数据结尾，这些数据中有变量吗？？？
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
				if (g_esp8266_focus[i](line))//line就是readFocus的三个函数的参数
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


uint32_t waitUntilRxStopped(uint32_t ms, uint16_t interval)//整体来说，ms是wait的最大时间，interval是两次模块返回间隔的最大时间；当MCU间隔ms时间没有接受数据时，返回0；
													//在没有等够ms时间内，当两次接收数据间隔时间大于inteval时，返回，返回值为剩余可等待时间timeOutTick - currentTick
{
	uint32_t timeOutTick = getTick() + ms;//只运行一次，所以运行后为定值，并不是三个变量都一样（都与getTick有关），有些变量是不断更新的，比如currentTick = getTick();
	uint32_t lastRxTick = getTick();
	uint32_t currentTick = getTick();
	uint16_t i = lastIndex(),j;
	while(timeOutTick - currentTick< (UINT32_MAX>>1))//相当于currentTick一直在更新，直到更新到timeOutTick小于currentTick为止
	{
		j = i;
		i = lastIndex();//lastIndex()=【ESP8266_BUFFER_SIZE-*(g_esp8266_dmaRemain)-1】& ESP8266_BUFFER_MASK;*(g_esp8266_dmaRemain)为接收到数据的数量；
										//i可以一定程度上表征数据接收数量？？？至于为何可这么表示，待求？？？
		if(i!=j)//如果有代码的接收，则buffer中为接收数据的空间发生变化，则i!=j,则更新lastRxTick
		{
			lastRxTick = getTick();
		}
		if (currentTick - (lastRxTick + interval) < (UINT32_MAX>>1))//由于currentTick一直更新，并且上次接收数据的时间也是一直更新的，只有当上次接收数据时间不再更新，
																																//也就是不再接收数据时，此情况才有效，所以，此code意思是两次代码间隔时间大于interval时，返回等待时间的剩余
		{
			return timeOutTick - currentTick;
		}
		currentTick = getTick();//这里是对currentTick的更新
	}
	return 0;
}

uint8_t waitUntilOk(uint32_t ms)//getTick是按周期获取的时钟时间      //此函数作用是确保指令发送成功，若成功则返回非零值，若不成功，则返回零值
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
			for (uint8_t i = 0; i< sizeof(g_esp8266_focus)/sizeof(focus_t); i++)//直接写成3不行吗？？？
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

static void reset(void)//如果模块重启前正在配置wifi，当wifi模块硬件重启后，使模块继续wifi的配置；若重启前没有配置wifi，则什么都不执行
{
	hw_reset();//注意g_esp8266_smartStarted是wifi配置的标志位，表示是的是当wifi配置时标志位为1，而不是表示当标志位为1时进行wifi配置，所以尽管模块硬重置，但标志位仍旧为1，只是
						 //wifi配置的函数已经中断了
	if (g_esp8266_isAtMode)//g_esp8266_isAtMode是什么标志？？？
	{
	//	setATMode();
	}
	if (g_esp8266_smartStarted)
	{
		uint32_t smartRemain = g_esp8266_smartTick - getTick();//注意这个重启hw_reset（）只是wifi模块的硬件重启，不会导致MCU丢失数据，所以重启前的wifi配置函数smart（）中的
																													 //配置时间ms依旧存在，也就是现在计算出的 smartRemain = g_esp8266_smartTick - getTick()，所以接下来再次运行smart（）时，运行esp8266_smart(smartRemain)即可
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
	if(g_esp8266_smartStarted)//也就是一直在检测smart过程是否超时，若超时，停止smart并且刷新g_esp8266_resetTick = getTick时间
	{
		if ((getTick() - g_esp8266_smartTick)<(UINT32_MAX>>1))//g_esp8266_smartTick=getTick+ms,从开始处就不再刷新，所以，这里表示的是smartlink过程超时
		{
			esp8266_smartStop();//注意，此句代码停止smart过程以后，会把g_esp8266_smartStarted置0
			reset();//由于esp8266_smartStop()会把g_esp8266_smartStarted置0，所以这句code只是刷新g_esp8266_resetTick的时刻
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
	reset();//如果模块重启前正在配置wifi，当wifi模块硬件重启后，使模块继续wifi的配置；若重启前没有配置wifi，则什么都不执行
	//Step 4: wait until the "ready" appeared.
	hw_delay(500);
	waitUntilRxStopped(5000,100);//MCU在等待时间ms内没有接受数据时，返回0；在没有等够ms时间内，当两次接收数据间隔时间大于inteval时，返回剩余可等待时间timeOutTick - currentTick
	readFocus();//?????分析接收到的数据
	//Step 5: check if in TM mode.
	printf("\n");
	hw_delay(1000);
	uint8_t test = lastIndex();//???
	char c = bufferLastChar();//????
	if (c != '\n' )
	{
		//Setp 5.1: if not in TM mode. Set the default properties.
		setATMode();//这是何意？？
	}
	setCommandWait(5000,"CWMODE_DEF","1",NULL);  //sta mode.   意思是：at+CWMODE_DEF=1
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
	g_esp8266_isAtMode = 0;//是否在透传模式下，已不用，自己看！！！？？？
//		reset();  // reset to enter tm mode.
//		waitUntilRxStopped(waitUntilBlankLine(5000),25);
//		readFocus();
}
uint8_t smart(int8_t type)//wifi配置函数
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
			return smart(type);//函数的自身嵌套，在type=0,1,2,3时，循环执行本函数,如果waitUntilOk有返回，则return1，如果无返回，则从3到1方式各执行一次
		}
	}
}
void esp8266_smart(uint32_t ms)//进行wifi配置smart(3)，并把标志位g_esp8266_smartStarted置1，设定配置时间ms
{
	if(smart(3))//依照3方式通讯配置wifi，ESP8266指令pdf上面没有相应的指令是因为这个pdf版本太低
	{
		g_esp8266_smartStarted = 1;//wifi配置的标志位
		g_esp8266_smartTick = getTick()+ms;//
		
	}
	readFocus();
}

void esp8266_smartStop(void)//停止smart过程（发送停止指令，并且等待5s，看是否发送成功）
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
