#include "rfid.h"

#define RFIDBUFFERSIZE	4

uint8_t RfidRxbuff = 0;
volatile uint8_t rfidRecFlag = 0;
volatile uint8_t rfididsave = 0;
uint8_t rfididsave_last = 0;
void rfid_init(void)
{
	HAL_UART_Receive_IT(&RFID_UART,&RfidRxbuff,1);
}

void RFID_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	rfididsave = RfidRxbuff;
	rfidRecFlag = 1;
	
	HAL_UART_Receive_IT(&RFID_UART,&RfidRxbuff,1);
}

uint8_t rfid_getsta(void)
{
	if(rfidRecFlag)
	{
		rfidRecFlag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t rfid_getid(void)
{
	return rfididsave;
}