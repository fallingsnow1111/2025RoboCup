#ifndef __IMU_H
#define __IMU_H

#include "delay.h"
#include "buzzer.h"
#include "main.h"
#include "usart.h"
#include <string.h>
extern DMA_HandleTypeDef hdma_usart2_rx;
#define IMU_BUFFER_SIZE 22
struct Imu
{
	float yaw;   	//左右
	float roll;		//侧翻
	float pitch;  	//抬头
	
	float yaw_set; 	//软校准后的左右
	float compensateZ;
};

volatile extern struct Imu imu;
extern uint8_t imu_rx_buf[IMU_BUFFER_SIZE];
extern uint8_t imu_rx_len;


float need2turn(float nowangle,float targetangle);
void mpuZreset(float sensorangle ,float referangle);


void gyro_init(void);
void gyro_set_0(void);
void gyro_set_reduce_90(void);
void gyro_set_plus_90(void);
void gyro_monitor(void);

#endif

