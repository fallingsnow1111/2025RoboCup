#ifndef __BUZZER_H
#define __BUZZER_H

#include "delay.h"
#include "main.h"

extern uint8_t buzzer_wrong_flag;
extern uint8_t buzzer_complete_flag;

void Buzzer_three_times(void);
void Buzzer_two_times(void);
void buzzer_task(void *pvParameters);
void buzzer_task_create(void);

#endif
