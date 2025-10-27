#ifndef __DELAY_H
#define __DELAY_H

//用户外设
#include "tim.h"
//核心
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

void delay_init(void);
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);

#endif
