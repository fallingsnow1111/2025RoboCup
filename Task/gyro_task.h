#ifndef __GYRO_H
#define __GYRO_H

#include "imu.h"
#include "Motor_Command.h"

#include "pid.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

extern uint8_t stop_flag;
extern uint8_t x_stop_flag;//x修正停止标志
extern uint8_t y_stop_flag;//y修正停止标志
extern float yaw_set_buf[10];

extern float yaw_set_arv;

void gyro_task(void *pvParameters);
void gyro_task_create(void);
extern TaskHandle_t gyro_task_handler;

void x_delay_task(void *pvParameters);
void y_delay_task(void *pvParameters);
void x_delay_task_create(void);
void y_delay_task_create(void);
extern TaskHandle_t x_delay_task_handler;
extern TaskHandle_t y_delay_task_handler;

#endif
