#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "usmart.h"
#include "stdio.h"
#include "qrcode.h"
#include "Motor_Command.h"
#include "string.h"
#include "usart.h"
#include "rfid.h"
#include "stepmotor_cmd.h"

void Serial_Init(void);
void Serial_Send(UART_HandleTypeDef *huart,uint8_t *cmd);

#endif
