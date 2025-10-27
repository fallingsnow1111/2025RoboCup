#ifndef __ARM_CMD_H__
#define __ARM_CMD_H__

#include "usart.h"

#define ARM_UART huart7

typedef struct _lobot_servo_ {  //
	uint8_t ID;
	uint16_t Position;
} LobotServo;

void cmd_action_group_run(uint8_t action_group,uint16_t number);
static void cmd_action_group_speed(uint8_t action_group,uint16_t speed);
void arm_action_group_run(uint8_t id);
void arm_action_group_run_noblock(uint8_t id);
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
uint8_t arm_isrespond(void);
void arm_init(void);
void ARM_UART_IDLECallback(UART_HandleTypeDef *huart);
void ARM_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void arm_setAngArray(uint16_t * ptr);
#endif
