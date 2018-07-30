#include "SN76489.h"

void SN_Init(GPIO_TypeDef * port)
{
	_port = port;
}

void SN_WriteDataPins(char data)
{
	//WE = PA09:0x100
	GPIOA->ODR |= 0x100; //WE HIGH

	Databus.PORT = _port->ODR;
	Databus.bytes[0] = data;
	_port->ODR = Databus.PORT;

	GPIOA->ODR &= ~(0x100); //WE HIGH
	Delay(14); //Delay 14 microseconds
	GPIOA->ODR |= 0x100; //WE HIGH
}

void SN_Reset()
{
	SN_WriteDataPins(0x9F);
	SN_WriteDataPins(0xBF);
	SN_WriteDataPins(0xDF);
	SN_WriteDataPins(0xFF);
}
