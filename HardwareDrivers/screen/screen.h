#ifndef __SCREEN_H
#define __SCREEN_H

//ºËÐÄ
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "qrcode.h"
#include "lcd.h"
#include "lcd_init.h"
void screen_init(void);
void screen_task(void *pvParameters);
void screen_task_create(void);
extern TaskHandle_t screen_task_handler;

#endif

