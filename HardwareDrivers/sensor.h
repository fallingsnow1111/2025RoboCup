#ifndef __SENSOR_H__
#define __SENSOR_H__
#include "stm32f7xx.h"
#include "usart.h"

/****红外传感器定义****/
/* 把红外当作车头看 */
/*		同侧		*/
/* 		L1 L2  R2 R1		*/
/* 	EX2 MID2 IN2 IN1 MID1 EX1	*/
/* 		start		*/

#define READ_R1_IR	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)
#define READ_R2_IR	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)
#define READ_L2_IR	HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define READ_L1_IR	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)
#define READ_start_IR	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)

#define READ_TK_IN1		HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_15)
#define READ_TK_IN2		HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)
#define READ_TK_MID1	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)
#define READ_TK_MID2	HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)
#define READ_TK_EX1	 	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)
#define READ_TK_EX2 	HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)


/*  		对侧				*/
/* 	  TK6 TK5 TK4 TK3 TK2 TK1 	*/
/********************/
#define READ_TK1	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)
#define READ_TK2	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)	
#define READ_TK3	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)
#define READ_TK4	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)
#define READ_TK5    HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)
#define READ_TK6	HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)

#define READ_PHOTOGATE    HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)
#define READ_COLOR_SELECT	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)


typedef enum SENOR{lir1 = 1,rir1,lir2,rir2,start,in1,in2,mid1,mid2,ex1,ex2,tk1,tk2,tk3,tk4,tk5,tk6,photogate,color_select}SENSOR_t;

GPIO_PinState read_iosensor(SENSOR_t sensor_id);

#endif
