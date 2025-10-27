#ifndef __LED_H
#define __LED_H

#include "delay.h"
#include "main.h"

void TogglePin_PB9(void);

void led_task(void *pvParameters);
void led_task_create(void);
extern TaskHandle_t led_task_handler;

#endif
