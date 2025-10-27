#ifndef __QRCODE_H__
#define __QRCODE_H__
#include "usart.h"

#define QRCODE_UART	huart4

void qrcode_init(void);
void QR_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t *qrcode_getsta(void);
uint8_t qrcode_getdata(void);
uint8_t qrcode_getRxbuff(void);
#endif
