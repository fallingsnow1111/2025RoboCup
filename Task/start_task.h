#ifndef __START_TASK_H
#define __START_TASK_H

//用户外设
#include "delay.h"
#include "usart.h"
#include "Motor_Command.h"
#include "led.h"
#include "imu.h"
#include "qrcode.h"
#include "screen.h"
#include "load2.h"
#include "openmv_cmd.h"
#include "arm_cmd.h"
//运行任务
#include "main_task.h"
//USMART
#include "usmart.h"
#include "usmart_str.h"
//核心
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t Init_Flag;

void start_task(void *pvParameters);
void start_task_create(void);
extern TaskHandle_t start_task_handler;

void load_task(void *pvParameters);
void load_task_create(void);

void user_init(void *pvParameters);
void user_init_create(void);
extern TaskHandle_t user_init_handler;

#endif

