#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H


//用户外设
#include "delay.h"
//#include "uart.h"
#include "start_task.h"
//核心
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
//电机控制
#include "Motor_Command.h"
#include "Motor_Run.h"
//其他外设
#include "sensor.h"
#include "rfid.h"
#include "Servo.h"

struct Main_Task_Fast_Mapping_inf{
	uint8_t run_mode;		//直行为1	平移为2	旋转为3
	
	float Task_angle;		//粗定位脉冲数
	float Turn_angle;		//旋转角度（旋转模式下不考虑粗定位脉冲数）
	int16_t Task_speed;	//任务进行速度
	//smooth_speed专有参数		
	uint8_t Speed_Acc;		//加速度
	float Accelerate;		//加速路程的比率
	uint8_t Speed_Dec;		//减速度
	float Decelerate;		//减速路程的比率
};

extern struct Main_Task_Fast_Mapping_inf Main_Task[50];

void main_task(void *pvParameters);
void main_task_create(void);
extern TaskHandle_t main_task_handler;

void Fast_Mapping_Run(struct Main_Task_Fast_Mapping_inf Task_inf);
void Fast_Mapping_Run_Judge(struct Main_Task_Fast_Mapping_inf Task_inf,uint8_t Target_Position);
void Judge_after_reaching_the_center(uint8_t Target_Position);

void Motor_stop(void);
void find_Disc(void);
void Disc_catch(void);
void Disc2Stairs(void);
void Stair_catch(void);
void find_bar_quad(void);
void clab_bar_quad(void);
void bar_edge(uint8_t edge);
void find_colu(void);
void ball_step_putback(uint8_t sta);
void Stairs2Circular(void);
void Pillar_catch(void);
void Circular(void);
void Blue_colu2stack(void);
void Blue_stack(void);
void BTrk09scor(void);
void Blue_go_home(void);
void Red_Main_Task(void);
void Blue_Main_Task(void);
void ALL_Main_Task(void);
void R_find_Disc(void);
void R_Disc2Stairs(void);
void Red_go_home(void);

#endif

