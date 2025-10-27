#ifndef __RFID_H__
#define __RFID_H__

#include "serial.h"

#define RFID_UART huart8

uint8_t rfid_getsta(void);
void rfid_init(void);
uint8_t rfid_getid(void);
void RFID_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
