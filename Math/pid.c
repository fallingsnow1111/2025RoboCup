#include "pid.h"

InitTypeDef_Position_PID X_Correct_Position_PID = {0};
InitTypeDef_Position_PID Y_Correct_Position_PID = {0};
InitTypeDef_Position_PID Gyro_Correct_Position_PID = {0};
InitTypeDef_Position_PID Camera_Correct_Position_PID = {0};
InitTypeDef_Position_PID Camera_X_Correct_Position_PID = {0};
InitTypeDef_Position_PID Camera_Y_Correct_Position_PID = {0};

InitTypeDef_Param_PID X_Correct_Param_PID = {0};
InitTypeDef_Param_PID Y_Correct_Param_PID = {0};
InitTypeDef_Param_PID Gyro_Correct_Param_PID = {0};
InitTypeDef_Param_PID Camera_Correct_Param_PID = {0};
InitTypeDef_Param_PID Camera_X_Correct_Param_PID = {0};
InitTypeDef_Param_PID Camera_Y_Correct_Param_PID = {0};

void PID_Init(void)
{	
	
	X_Correct_Param_PID.Kp = 40;
	X_Correct_Param_PID.Ki = 0.05;
	X_Correct_Param_PID.Kd = 0;
	
	Y_Correct_Param_PID.Kp = 100;
	Y_Correct_Param_PID.Ki = 0;
	Y_Correct_Param_PID.Kd = 0.5;
	
	Gyro_Correct_Param_PID.Kp = 100;//200;
	Gyro_Correct_Param_PID.Ki = 0.3;
	Gyro_Correct_Param_PID.Kd = 0.5;

}//在这里修改PID的参数


/*位置式PID，带积分限幅*/
float Position_PID(InitTypeDef_Position_PID *Position_PID,InitTypeDef_Param_PID *Param_PID)
{
	float Pout,Iout,Dout = 0;
	float differential = 0;
	
	Position_PID -> error = Position_PID -> target - Position_PID -> measure;
	Position_PID -> integral += Position_PID -> error;
	differential = Position_PID ->error - Position_PID -> last_error;
	
	if(Position_PID -> integral > PID_INTEGRAL_MAX)
		Position_PID -> integral = PID_INTEGRAL_MAX;
	if(Position_PID -> integral < -PID_INTEGRAL_MAX)
		Position_PID -> integral = -PID_INTEGRAL_MAX;//积分限幅
	
	Pout = Param_PID -> Kp * Position_PID -> error;
	Iout = Param_PID -> Ki * Position_PID -> integral;
	Dout = Param_PID -> Kd * differential;
	
	Position_PID -> output = Pout + Iout + Dout;
	
	Position_PID -> last_error = Position_PID ->error;
	
	return Position_PID -> output;
}

/*增量式*/
float Incremental_PID(InitTypeDef_Incremental_PID *Incremental_PID,InitTypeDef_Param_PID *Param_PID)
{
	float Pout,Iout,Dout = 0;
	float integral,differential = 0;
	
	Incremental_PID -> error = Incremental_PID -> target - Incremental_PID -> measure;
	integral = Incremental_PID -> error;
	differential = Incremental_PID -> error - 2 * Incremental_PID -> last_error + Incremental_PID -> last2_error;
	
	Pout = Param_PID -> Kp * (Incremental_PID -> error - Incremental_PID -> last_error);
	Iout = Param_PID -> Ki * integral;
	Dout = Param_PID -> Kd * differential;
	
	Incremental_PID -> output += Pout + Iout + Dout;
	
	Incremental_PID -> last_error = Incremental_PID -> error;
	
	return Incremental_PID -> output;
}


