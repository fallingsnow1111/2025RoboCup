#ifndef __JETSON_H
#define __JETSON_H

//核心
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include "Servo.h"
#include "screen.h"
#include <string.h>
#include <stdlib.h>

//以下定义用于模式切换发送

extern uint8_t RED_Mode[];
extern uint8_t BLUE_Mode[];
extern DMA_HandleTypeDef hdma_usart6_rx;

extern _Bool catch_flag;
extern _Bool loose_flag;

void Claw_loose(void);
void Claw_catch(void);
void jetson_rx_init(void);
void Jetson_Mode_Change(uint8_t*sign);
void BLUE_Mode_Change(void);
void RED_Mode_Change(void);


#endif