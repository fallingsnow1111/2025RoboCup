#ifndef __SERVO_H
#define __SERVO_H

#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "buzzer.h"
#include "usart.h"

#define start_init 0
#define start_init 0
#define li_zhuang_detect 2
#define li_zhuang_catch  3
void servo_control(uint8_t servo_cmd_id);

#endif
