#include "arm_cmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

#define ACTIONSPEED 180
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
//宏函数 获得A的高八位

#define ARMBUFFERSIZE 7

#define SERVO_NUM 5

uint8_t arm_rxbuff[ARMBUFFERSIZE] = {0};
volatile uint8_t arm_rxflag = 0;

uint16_t action_speed_g0[9] = {100,100};			//出发与回家姿态
uint16_t action_speed_g1[9] = {150,180};			//圆盘机
uint16_t action_speed_g3[9] = {150,150,150,150,150,150};//阶梯
uint16_t action_speed_g4[9] = {180,150};					//立桩
uint16_t action_speed_g5[9] = {100,180,150,100,180,150,100,180,150}; //倒舵
uint16_t action_speed_g7[9] = {180,180,180,180,180,180,180,180,180}; //放球
uint16_t action_speed_g11[9] = {180,100};

LobotServo servos[SERVO_NUM];

void arm_init(void)
{
	HAL_UART_Receive_IT(&ARM_UART,arm_rxbuff,ARMBUFFERSIZE);
	for(uint8_t i = 0;i<SERVO_NUM;i++)
	{
		servos[i].ID = i;
	}
}

void arm_setAngArray(uint16_t * ptr)
{
	for(uint8_t i = 0;i < SERVO_NUM;i++)
	{
		servos[i].Position = ptr[i];
	}
	moveServosByArray(servos,SERVO_NUM,500);
}

void cmd_action_group_run(uint8_t action_group,uint16_t number)
{
    uint8_t command[7];
    command[0] = 0x55;
    command[1] = 0x55;
    command[2] = 0x05;
    command[3] = 0x06;
    command[4] = action_group;
    command[5] = number&0XFF;
    command[6] = (number>>8)&0XFF;
    HAL_UART_Transmit(&ARM_UART,command,7,0xffff);
}

void cmd_action_group_speed(uint8_t action_group,uint16_t speed)
{
    uint8_t command[7];
    command[0] = 0x55;
    command[1] = 0x55;
    command[2] = 0x05;
    command[3] = 0x0B;
    command[4] = action_group;
    command[5] = speed&0XFF;
    command[6] = (speed>>8)&0XFF;
    HAL_UART_Transmit(&ARM_UART,command,7,0xffff);  //发送动作组编号和执行速度
}

//from board examples
void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint8_t LobotTxBuf[7+SERVO_NUM*3];
	if (Num < 1 || Num > 32 || !(Time > 0)) {
		return;                                          //舵机数不能为零和大与32，时间不能为零
	}
	LobotTxBuf[0] = LobotTxBuf[1] = 0x55;      //填充帧头
	LobotTxBuf[2] = Num * 3 + 5;                       //数据长度 = 要控制舵机数*3+5
	LobotTxBuf[3] = 0x03;                    //填充舵机移动指令
	LobotTxBuf[4] = Num;                               //要控制的舵机个数
	LobotTxBuf[5] = GET_LOW_BYTE(Time);                //取得时间的低八位
	LobotTxBuf[6] = GET_HIGH_BYTE(Time);               //取得时间的高八位

	for (i = 0; i < Num; i++) {                       //循环填充舵机ID和对应目标位置
		LobotTxBuf[index++] = servos[i].ID;              //填充舵机ID
		LobotTxBuf[index++] = GET_LOW_BYTE(servos[i].Position); //填充目标位置低八位
		LobotTxBuf[index++] = GET_HIGH_BYTE(servos[i].Position);//填充目标位置高八位
	}
	HAL_UART_Transmit(&ARM_UART,LobotTxBuf, LobotTxBuf[2] + 2,0xffff);             //发送
}

void arm_action_group_run(uint8_t id)
{
	uint16_t action_speed = 100;
	switch(id/10)
	{
		case 0:
		{
			action_speed = action_speed_g0[id%10];
			break;
		}
		case 1:
		{
			action_speed = action_speed_g1[id%10];
			break;
		}
		case 3:
		{
			action_speed = action_speed_g3[id%10];
			break;
		}
		case 4:
		{
			action_speed = action_speed_g4[id%10];
			break;
		}
		case 5:
		{
			action_speed = action_speed_g5[id%10];
			break;
		}
		case 7:
		{
			action_speed = action_speed_g7[id%10];
			break;
		}
		case 11:
		{
			action_speed = action_speed_g11[id%10];
			break;
		}
		default:	//incorrect action num!
		{
			action_speed = 0;
		}
	}
	if(action_speed)
	{
		cmd_action_group_speed(id,action_speed);
		cmd_action_group_run(id,1);
		while(arm_isrespond() == 0)
		{
			vTaskDelay(10);
		}
	}

}

void arm_action_group_run_noblock(uint8_t id)
{
	cmd_action_group_speed(id,ACTIONSPEED);
	vTaskDelay(20);
	cmd_action_group_run(id,1);
}

void ARM_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if((arm_rxbuff[0] == 0x55) && 
		(arm_rxbuff[1] == 0x55) && 
		(arm_rxbuff[2] == 0x05) &&
		(arm_rxbuff[3] == 0x08))
		{
			arm_rxflag = 1;
		}
		memset(arm_rxbuff,0,sizeof(arm_rxbuff));
		HAL_UART_Receive_IT(&ARM_UART,arm_rxbuff,ARMBUFFERSIZE);
}
/*
void ARM_UART_IDLECallback(UART_HandleTypeDef *huart)		//舍弃空闲中断
{
	HAL_UART_DMAStop(&ARM_UART);
	if((arm_rxbuff[0] == 0x55) && 
	(arm_rxbuff[1] == 0x55) && 
	(arm_rxbuff[2] == 0x05) &&
	(arm_rxbuff[3] == 0x08))
	{
		arm_rxflag = 1;
		memset(arm_rxbuff,0,sizeof(arm_rxbuff));
	}
	HAL_UART_Receive_DMA(&ARM_UART,arm_rxbuff,ARMBUFFERSIZE);
}
*/

uint8_t arm_isrespond(void)
{
	if(arm_rxflag == 1)
	{
		arm_rxflag = 0;
		return 1;
	}
	return 0;
}
