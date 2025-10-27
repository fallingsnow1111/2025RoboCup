#ifndef __ELECTRIC_SWITCH_H
#define __ELECTRIC_SWITCH_H

//ºËÐÄ
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "gpio.h"
#include "buzzer.h"
extern uint8_t ready_flag;
void switch_Init(void);
void IsReady(void);

#endif