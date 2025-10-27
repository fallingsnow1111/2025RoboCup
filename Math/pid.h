#ifndef __PID_H
#define __PID_H
#include "stm32f7xx_hal.h"

/*积分最大限幅*/
#define PID_INTEGRAL_MAX 100
typedef struct{
	float target;
	float measure;
	float error;
	float last_error;
	float integral;
	
	float output;
}InitTypeDef_Position_PID;

typedef struct{
	float target;
	float measure;
	float error;
	float last_error;
	float last2_error;
	
	float output;
}InitTypeDef_Incremental_PID;

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	
}InitTypeDef_Param_PID;

extern InitTypeDef_Position_PID X_Correct_Position_PID;
extern InitTypeDef_Position_PID Y_Correct_Position_PID;
extern InitTypeDef_Position_PID Gyro_Correct_Position_PID;
extern InitTypeDef_Position_PID Camera_Correct_Position_PID;
extern InitTypeDef_Position_PID Camera_X_Correct_Position_PID;
extern InitTypeDef_Position_PID Camera_Y_Correct_Position_PID;

extern InitTypeDef_Param_PID X_Correct_Param_PID;
extern InitTypeDef_Param_PID Y_Correct_Param_PID;
extern InitTypeDef_Param_PID Gyro_Correct_Param_PID;
extern InitTypeDef_Param_PID Camera_Correct_Param_PID;
extern InitTypeDef_Param_PID Camera_X_Correct_Param_PID;
extern InitTypeDef_Param_PID Camera_Y_Correct_Param_PID;

void PID_Init(void);
float Position_PID(InitTypeDef_Position_PID *Position_PID,InitTypeDef_Param_PID *Param_PID);
float Incremental_PID(InitTypeDef_Incremental_PID *Incremental_PID,InitTypeDef_Param_PID *Param_PID);

#endif







