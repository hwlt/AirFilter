#ifndef DUST_H__
#define DUST_H__

#include "stm32f0xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"

typedef struct
{
	uint32_t PM1D0V;
	uint32_t PM2D5V;
	uint32_t PM10V;	
	uint32_t PC0D3V;
	uint32_t PC1D0V;
	uint32_t PC2D5V;
	uint32_t PC5D0V;
	uint32_t PC10V;	
}dustvalue_t;


void UART2_Config_B2(char *stringToSend);
void PM2D5_init(void);
bool get_dust_data(void);
bool dust_receiveData(int32_t retries);
void dust_update(void);

#endif
