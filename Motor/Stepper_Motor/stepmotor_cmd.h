#ifndef __STEPMOTOR_CMD_H__
#define __STEPMOTOR_CMD_H__

#include "usart.h"

#define STEPMOTOR_UART huart5

void stpmot_init(void);
void STPMOT_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
uint8_t stpmot_getsta(void);
void stpmot_to_degree(uint32_t degree,uint8_t direct);
void stpmot_enable(uint8_t sta);
void stpmot_find_zero(void);
void stpmot_stop_find_zero(void);
uint8_t stpmot_find_zero_sta(void);
void STPMOT_UART_IDLECallback(UART_HandleTypeDef *huart);
uint8_t stpmot_work_sta(void);
void STEPMOTOR_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
