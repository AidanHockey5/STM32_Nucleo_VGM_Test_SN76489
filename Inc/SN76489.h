#ifndef SN76489_H_
#define SN76489_H_

#include "stdint.h"
#include "tm_stm32_delay.h"

union _BUS //Split up the 32-bit data bus port register into four, 8-bit chunks. Write to bytes[0] for data bus
{
	uint32_t PORT;
	unsigned char bytes[4];
} Databus;

GPIO_TypeDef * _port;
uint16_t _we;

void SN_Init(GPIO_TypeDef * port);
void SN_WriteDataPins(char data);
void SN_Reset();

#endif
