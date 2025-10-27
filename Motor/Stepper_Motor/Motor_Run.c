#include "Motor_Run.h"

//轮径80mm 周长=2pi*75/2=235.62 
#define WHEEL_CIRCUMFERENCE  235.62		//轮子周长(mm)
#define PULSES_PER_ROTATION  3200		//每圈的脉冲数

#define Correct_Speed_MAX 500	//直行、平移最大修正值

#define Rot_Correct_Speed_MAX 2500//最大旋转修正值
#define Rot_Correct_Omega_rate 3//最大旋转速度倍率

#define Camera_Correct_Low_Speed 20
#define Camera_Correct_Speed_MAX 2000//摄像头纠正最大修正值
#define Camera_Correct_X_Y_Speed_MAX 100 //摄像头纠正最大修正值
#define Camera_Correct_Stop 1//停止的精度范围
#define Camera_Correct_Sepeed_Change_rate 3//停止的精度范围

#define PI 3.1415926


uint32_t Distance_to_Pluse_Num(float dis){
	uint32_t Pluse_num = 0;
    Pluse_num = (uint32_t)( (dis*PULSES_PER_ROTATION)/WHEEL_CIRCUMFERENCE + 0.5 );//四舍五入
	return Pluse_num;
}

uint32_t Angle_to_Pluse_Num(float agl){
	
}

void Stepping_Motor_Param_change(	struct Stepping_Motor_Param* Motor_Param,
									uint32_t Pluse_num,
									uint8_t dir,
									uint16_t speed,
									uint8_t acc
									){
		
		Motor_Param->Pulse_Number = Pluse_num;								
		Motor_Param->Motor_Dir = dir;
		Motor_Param->Motor_Speed = speed;
		Motor_Param->Motor_Acc = acc;
	}

void Motor_translate_run(float distance,uint8_t dir,uint16_t speed,uint8_t acc){
	uint16_t Pluse_num_temp = 0;
	Pluse_num_temp = Distance_to_Pluse_Num(distance);
	//平移行驶电机的方向
	uint8_t dir_1 = dir;	uint8_t dir_2 = dir;
	uint8_t dir_3 = !dir;	uint8_t dir_4 = !dir;
	//修改电机参数结构体
	Stepping_Motor_Param_change(&Motor_1,Pluse_num_temp,dir_1,speed,acc);
	Stepping_Motor_Param_change(&Motor_2,Pluse_num_temp,dir_2,speed,acc);
	Stepping_Motor_Param_change(&Motor_3,Pluse_num_temp,dir_3,speed,acc);
	Stepping_Motor_Param_change(&Motor_4,Pluse_num_temp,dir_4,speed,acc);
	//控制电机运行
	Motor_Speed_Pulse_Control(Motor_4);
	Motor_Speed_Pulse_Control(Motor_3);
	Motor_Speed_Pulse_Control(Motor_1);
	Motor_Speed_Pulse_Control(Motor_2);
	
}

/*************************************************/
/*												 */
/*												 */
/*			   	    PID＋差速行驶				 */
/*												 */
/*												 */
/*************************************************/
//位置闭环PID 采用陀螺仪、速度控制
void Motor_gyro_straight_run(uint8_t dir,uint16_t speed,uint8_t acc){
	//修正速度
	int16_t Correct_speed = 0;
	//直线行驶PID参数
	Gyro_Correct_Param_PID.Kp = 5.2;
	Gyro_Correct_Param_PID.Ki = 0;
	Gyro_Correct_Param_PID.Kd = 2.5;
	//电机速度修改中间量
	int16_t Motor_L1 = 0;
	int16_t Motor_L2 = 0;
	int16_t Motor_R1 = 0;
	int16_t Motor_R2 = 0;
	//直线行驶电机的方向
	uint8_t dir_1 = dir;	uint8_t dir_2 = !dir;
	uint8_t dir_3 = dir;	uint8_t dir_4 = !dir;
	//修改初始电机参数结构体
	Stepping_Motor_Param_change(&Motor_1,0,dir_1,speed,acc);
	Stepping_Motor_Param_change(&Motor_2,0,dir_2,speed,acc);
	Stepping_Motor_Param_change(&Motor_3,0,dir_3,speed,acc);
	Stepping_Motor_Param_change(&Motor_4,0,dir_4,speed,acc);
	//设置直行的角度目标
	Gyro_Correct_Position_PID.target=0;
	//控制电机运行
	Motor_Speed_Control(&Motor_1);
	Motor_Speed_Control(&Motor_2);
	Motor_Speed_Control(&Motor_3);
	Motor_Speed_Control(&Motor_4);
	
	while(1){
		//往左转 yaw_set>0 往右转 yaw_set<0
		Gyro_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&Gyro_Correct_Position_PID,&Gyro_Correct_Param_PID);
		//设置修正限幅
		if(Correct_speed >= Correct_Speed_MAX) Correct_speed = Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//直线形式速度修正		
		if(Correct_speed <= Correct_Speed_MAX && Correct_speed >= -Correct_Speed_MAX){
			Motor_L1 = speed - Correct_speed;
			Motor_L2 = speed - Correct_speed;
			
			Motor_R1 = speed + Correct_speed;
			Motor_R2 = speed + Correct_speed;	
		}
		
		//设置输出限幅
		if(Motor_L1>=speed*2){Motor_L1=speed*2;} if(Motor_L1<=0){Motor_L1=0;}
		if(Motor_L2>=speed*2){Motor_L2=speed*2;} if(Motor_L2<=0){Motor_L2=0;}
		if(Motor_R1>=speed*2){Motor_R1=speed*2;} if(Motor_R1<=0){Motor_R1=0;}
		if(Motor_R2>=speed*2){Motor_R2=speed*2;} if(Motor_R2<=0){Motor_R2=0;}
		//修改电机速度
		Motor_1.Motor_Speed = Motor_L1; 
		Motor_2.Motor_Speed = Motor_R1; 
		Motor_3.Motor_Speed = Motor_L2; 
		Motor_4.Motor_Speed = Motor_R2; 
		//控制电机运行
		Motor_Speed_Control(&Motor_1);
		Motor_Speed_Control(&Motor_2);
		Motor_Speed_Control(&Motor_3);
		Motor_Speed_Control(&Motor_4);		
	}	
}

void Motor_gyro_translate_run(uint8_t dir,uint16_t speed,uint8_t acc){
	//修正速度
	int16_t Correct_speed = 0;	
	//平移行驶PID参数
	Gyro_Correct_Param_PID.Kp = 1.05;
	Gyro_Correct_Param_PID.Ki = 0;
	Gyro_Correct_Param_PID.Kd = 0.1;	
	//电机速度修改中间量
	int16_t Motor_L1 = 0;
	int16_t Motor_L2 = 0;
	int16_t Motor_R1 = 0;
	int16_t Motor_R2 = 0;	
	//平移行驶电机的方向
	uint8_t dir_1 = dir;	uint8_t dir_2 = dir;
	uint8_t dir_3 = !dir;	uint8_t dir_4 = !dir;	
	//修改初始电机参数结构体
	Stepping_Motor_Param_change(&Motor_1,0,dir_1,speed,acc);
	Stepping_Motor_Param_change(&Motor_2,0,dir_2,speed,acc);
	Stepping_Motor_Param_change(&Motor_3,0,dir_3,speed,acc);
	Stepping_Motor_Param_change(&Motor_4,0,dir_4,speed,acc);	
	//设置平移的角度目标
	Gyro_Correct_Position_PID.target=0;	
	//控制电机运行
	Motor_Speed_Control(&Motor_1);
	Motor_Speed_Control(&Motor_2);
	Motor_Speed_Control(&Motor_3);
	Motor_Speed_Control(&Motor_4);
	
	while(1){
	   	//往左转 yaw_set>0 往右转 yaw_set<0
		Gyro_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&Gyro_Correct_Position_PID ,&Gyro_Correct_Param_PID);
		//设置修正限幅
		if(Correct_speed >= Correct_Speed_MAX) Correct_speed = Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//平移形式速度修正限幅
		if(Correct_speed <= Correct_Speed_MAX && Correct_speed >= -Correct_Speed_MAX){
			Motor_L1 = speed - Correct_speed;
			Motor_L2 = speed + Correct_speed;
			
			Motor_R1 = speed - Correct_speed;
			Motor_R2 = speed + Correct_speed;	
		}
		//设置输出限幅
		if(Motor_L1>=speed*2){Motor_L1=speed*2;} if(Motor_L1<=0){Motor_L1=0;}
		if(Motor_L2>=speed*2){Motor_L2=speed*2;} if(Motor_L2<=0){Motor_L2=0;}
		if(Motor_R1>=speed*2){Motor_R1=speed*2;} if(Motor_R1<=0){Motor_R1=0;}
		if(Motor_R2>=speed*2){Motor_R2=speed*2;} if(Motor_R2<=0){Motor_R2=0;}
		//修改电机速度
		Motor_1.Motor_Speed = Motor_L1; 
		Motor_2.Motor_Speed = Motor_R1; 
		Motor_3.Motor_Speed = Motor_L2; 
		Motor_4.Motor_Speed = Motor_R2; 
		//控制电机运行
		Motor_Speed_Control(&Motor_1);
		Motor_Speed_Control(&Motor_2);
		Motor_Speed_Control(&Motor_3);
		Motor_Speed_Control(&Motor_4);
	}
}

/*************************************************/
/*												 */
/*												 */
/*			     麦克纳姆轮解算行驶       	     */
/*												 */
/*												 */
/*************************************************/
//电机速度修改中间量
	int16_t Motor_L1 = 0;
	int16_t Motor_L2 = 0;
	int16_t Motor_R1 = 0;
	int16_t Motor_R2 = 0;
	uint8_t dir_1=0;
	uint8_t dir_3=0;
	uint8_t dir_2=0;
	uint8_t dir_4=0;

void Motor_mecanum_wheel_speed_run(int16_t X_speed,int16_t Y_speed,int16_t Omega,uint8_t acc){

	//麦克纳姆轮解算四个轮子的速度
	Motor_L1 = Mecanum_Wheel_Solve(&Motor_1,X_speed,Y_speed,Omega);
	Motor_L2 = Mecanum_Wheel_Solve(&Motor_3,X_speed,Y_speed,Omega);
	Motor_R1 = Mecanum_Wheel_Solve(&Motor_2,X_speed,Y_speed,Omega);
	Motor_R2 = Mecanum_Wheel_Solve(&Motor_4,X_speed,Y_speed,Omega);
	//电机的方向(取符号位)
	 dir_1 = (uint8_t)(Motor_L1 >> 15);	 dir_2 = (uint8_t)(Motor_R1 >> 15);
	 dir_3 = (uint8_t)(Motor_L2 >> 15);	 dir_4 = (uint8_t)(Motor_R2 >> 15);
	//电机的速度(取低12位)
	if(dir_1 == 0xff)Motor_L1 = ~Motor_L1 + 1;if(dir_2 == 0xff)Motor_R1 = ~Motor_R1 + 1;
	if(dir_3 == 0xff)Motor_L2 = ~Motor_L2 + 1;if(dir_4 == 0xff)Motor_R2 = ~Motor_R2 + 1;
	Motor_L1 =(uint16_t)(Motor_L1 &( 0x0f<<8 | 0xff ));Motor_R1 =(uint16_t)(Motor_R1 &( 0x0f<<8 | 0xff ));
	Motor_L2 =(uint16_t)(Motor_L2 &( 0x0f<<8 | 0xff ));Motor_R2 =(uint16_t)(Motor_R2 &( 0x0f<<8 | 0xff ));
	//修改电机参数结构体
	Stepping_Motor_Param_change(&Motor_1,0,dir_1,Motor_L1,acc);
	Stepping_Motor_Param_change(&Motor_2,0,dir_2,Motor_R1,acc);
	Stepping_Motor_Param_change(&Motor_3,0,dir_3,Motor_L2,acc);
	Stepping_Motor_Param_change(&Motor_4,0,dir_4,Motor_R2,acc);
	//控制电机运行
	Motor_Speed_Control(&Motor_1);
	Motor_Speed_Control(&Motor_2);
	Motor_Speed_Control(&Motor_3);
	Motor_Speed_Control(&Motor_4);
	Motor_Run();
}

/*************************************************/
/*												 */
/*												 */
/*			   麦克纳姆轮解算＋PID				 */
/*												 */
/*												 */
/*************************************************/
void Motor_mecanum_wheel_pid_rot_run(float angle,uint8_t acc){
	//防止超时
	uint8_t tim_flag = 0;//定时任务开启标志
	//修正速度
	int16_t Correct_speed = 0;
	//设置平移的角度目标
	Gyro_Correct_Position_PID.target = angle;
	while(1)
	{
		//解算修正值
		Gyro_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&Gyro_Correct_Position_PID,&Gyro_Correct_Param_PID);
		//设置修正值限幅
		if(Correct_speed >=  Rot_Correct_Speed_MAX)Correct_speed =  Rot_Correct_Speed_MAX;
		if(Correct_speed <= -Rot_Correct_Speed_MAX)Correct_speed = -Rot_Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(0,0,Correct_speed,acc);
		
		/************退出旋转校正 设定*************/
		if((imu.yaw_set >= Gyro_Correct_Position_PID.target - 0.0001 &&
		    imu.yaw_set <= Gyro_Correct_Position_PID.target + 0.0001)
			||stop_flag == 1 )//在目标角度+-0.001°附近退出 或 超时的停止标志位置1时退出
		{	
			if(stop_flag == 1){
				Motor_mecanum_wheel_speed_run(0,0,0,acc);
				Gyro_Correct_Position_PID.integral = 0;
				if(angle >= 0) gyro_set_reduce_90();
				if(angle <= 0) gyro_set_plus_90();
				stop_flag = 0;//停止标志位复位
				break;
			}
			else
			{
				if(tim_flag == 1)   vTaskDelete(gyro_task_handler);
				Motor_mecanum_wheel_speed_run(0,0,0,acc);
				Gyro_Correct_Position_PID.integral = 0;
				if(angle >= 0) gyro_set_reduce_90();
				if(angle <= 0) gyro_set_plus_90();
				break;
			}
		}
		   
		//进入目标角度+-5°时，开启定时任务(仅开启一次)
		if((imu.yaw_set >= Gyro_Correct_Position_PID.target - 5 &&
		    imu.yaw_set <= Gyro_Correct_Position_PID.target + 5)&&
			tim_flag == 0)
		{
			tim_flag = 1;//定时任务开启标志位置一
			gyro_task_create();//该程序用于定时100ms，使stop_flag置1
		}
		/************退出旋转校正 设定*************/
	}
}



void Motor_mecanum_wheel_pid_straight_run(float Motor_Angle,int16_t X_speed,uint8_t acc){
	//修正速度
	int16_t Correct_speed = 0;
	//设置直线的角度目标
	X_Correct_Position_PID.target = 0;
	while(1)
	{
		//解算修正值
		X_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(X_speed,0,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
		//读取电机1输入脉冲 进行粗定位
		if(Motor1_inf.Rotted_Angle >= Motor_Angle){
			Motor_mecanum_wheel_speed_run(0,0,0,acc);
			break;			
		}
	}
	
	Set_Motors_Zero(&Motor_1);
}


void Motor_mecanum_wheel_pid_translate_run(float Motor_Angle,int16_t Y_speed,uint8_t acc){
	
	//修正速度
	int16_t Correct_speed = 0;
	//设置直线的角度目标
	Y_Correct_Position_PID.target = 0;
	while(1)
	{	
		//解算修正值
		Y_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&Y_Correct_Position_PID,&Y_Correct_Param_PID);
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(0,Y_speed,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
		//读取电机1输入脉冲 进行粗定位
		if(Motor1_inf.Rotted_Angle >= Motor_Angle){
			Motor_mecanum_wheel_speed_run(0,0,0,acc);
			break;			
		}
	}
	Set_Motors_Zero(&Motor_1);
}

void Motor_mecanum_wheel_advance_at_an_angle(float Motor_Angle,int16_t X_speed,int16_t Y_speed,uint8_t acc){
		//修正速度
	int16_t Correct_speed = 0;
	//设置直线的角度目标
	Y_Correct_Position_PID.target = 0;
	
	while(1)
	{	
		//解算修正值
		Y_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&Y_Correct_Position_PID,&Y_Correct_Param_PID);
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(X_speed,Y_speed,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
		//读取电机1输入脉冲 进行粗定位
		if(Motor1_inf.Rotted_Angle >= Motor_Angle){
			Motor_mecanum_wheel_speed_run(0,0,0,acc);
			break;			
		}
	}
	Set_Motors_Zero(&Motor_1);
}



float oblique_dis;
float cos_alpha ;
float sin_alpha ;
float tan_alpha ;
void Motor_mecanum_wheel_pid_oblique_run(float xdistance,float ydistance,int16_t oblique_speed,uint8_t acc){
	//修正角度
	 oblique_dis = sqrt(pow(xdistance,2)+pow(ydistance,2));
	 cos_alpha = fabs(xdistance/oblique_dis);
	 sin_alpha = fabs(ydistance/oblique_dis);
	 tan_alpha = fabs(ydistance/xdistance);
	//修正速度
	int16_t Correct_speed = 0;
	//设置直线的角度目标
	X_Correct_Position_PID.target = 0;
	Y_Correct_Position_PID.target = 0;
	float xangle = (xdistance / (3.14*75)) *360;
	float yangle = (ydistance / (3.14*75)) *360;
	while(1)
	{
		//解算修正值
		X_Correct_Position_PID.measure = imu.yaw_set;
		Y_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
		Correct_speed=(int16_t)Position_PID(&Y_Correct_Position_PID,&Y_Correct_Param_PID);
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(oblique_speed*cos_alpha,oblique_speed*sin_alpha,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
		Get_Motor_Rotted_Angle_Fun(&Motor_2,&Motor2_inf);
		if((ydistance>=0&&xdistance>=0)||(ydistance<=0&&xdistance<=0))
		{
			if(ydistance>=0&&xdistance>=0)
			{
			//读取电机2输入脉冲 进行粗定位
				if(Motor2_inf.Rotted_Angle*cos_alpha >= xangle && Motor2_inf.Rotted_Angle*sin_alpha >= yangle){
					Motor_mecanum_wheel_speed_run(0,0,0,acc);
					break;			
				}
		  }
			else if(ydistance<0&&xdistance<0)
			{
				if(Motor2_inf.Rotted_Angle*cos_alpha >= -xangle && Motor2_inf.Rotted_Angle*sin_alpha >= -yangle){
					Motor_mecanum_wheel_speed_run(0,0,0,acc);
					break;			
				}
			}
		}
		else if((ydistance<=0&&xdistance>=0)||(ydistance>=0&&xdistance<=0))
		{
			
			if(ydistance<=0&&xdistance>=0)
			{
					//读取电机1输入脉冲 进行粗定位
					if(Motor1_inf.Rotted_Angle*cos_alpha >= xangle && Motor1_inf.Rotted_Angle*sin_alpha >= -yangle){
						Motor_mecanum_wheel_speed_run(0,0,0,acc);
						break;			
					}
			}
			else if(ydistance>=0&&xdistance<=0)
			{
					if(Motor1_inf.Rotted_Angle*cos_alpha >= -xangle && Motor1_inf.Rotted_Angle*sin_alpha >= yangle){
						Motor_mecanum_wheel_speed_run(0,0,0,acc);
						break;			
					}
			}
		}
	}
	Set_Motors_Zero(&Motor_1);
	Set_Motors_Zero(&Motor_2);
}




#define Brake_Speed 700
void Motor_mecanum_wheel_pid_straight_run_smooth(float Motor_Angle,int16_t X_speed,uint8_t acc,uint8_t Speed_Acc,float Accelerate,uint8_t Speed_Dec,float Decelerate){
	//修正速度
	float correct_gain = 5.0f;
	int16_t Correct_speed = 0;
	int16_t Inc_speed = 0;
	//设置直线的角度目标
	X_Correct_Position_PID.target = 0;
	//加速运动
	while(1){
		if(X_speed >= 0){
			if(Inc_speed < X_speed){Inc_speed += Speed_Acc;}
			else{Inc_speed = X_speed;}
		}
		else
		{
			if(Inc_speed > X_speed){Inc_speed -= Speed_Acc;}
			else{Inc_speed = X_speed;}
		}
		//解算修正值
		X_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)(correct_gain * Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID));
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);			
		//移动至路程的0.35时进行匀速
		if(Motor1_inf.Rotted_Angle >= Accelerate*Motor_Angle){break;}
		vTaskDelay(10);
	}
	//匀速运动
	while(1){
		//解算修正值
		X_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)(correct_gain * Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID));
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);			
		//移动至路程的0.8时进行减速
		if(Motor1_inf.Rotted_Angle >= Decelerate*Motor_Angle){break;}
		vTaskDelay(10);
	}
	while(1){
		if(X_speed >= 0){
			if(Inc_speed > Brake_Speed){Inc_speed -= Speed_Dec;}
			else{Inc_speed = Brake_Speed;}
		}
		else{
			if(Inc_speed < -Brake_Speed){Inc_speed += Speed_Dec;}
			else{Inc_speed = -Brake_Speed;}
		}
		//解算修正值
		X_Correct_Position_PID.measure = imu.yaw_set;
		Correct_speed=(int16_t)(correct_gain * Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID));
		//设置修正值限幅
		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
		//控制电机解算并运行
		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
		//读取电机1的转过的角度
		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
		if(Motor1_inf.Rotted_Angle >= Motor_Angle){
			Motor_mecanum_wheel_speed_run(0,0,0,acc);
			break;			
		}
		vTaskDelay(10);
	}
	Set_Motors_Zero(&Motor_1);
}

int16_t Scan_Speed = 300 ;//200

//void Motor_mecanum_wheel_pid_straight_run_smooth_scan(float Motor_Angle,int16_t X_speed,uint8_t acc,uint8_t Speed_Acc,float Accelerate,uint8_t Speed_Dec,float Decelerate){
//	
//	Motor_Angle=(Motor_Angle/(3.14*80)) *360;
//	
//	//修正速度
//	int16_t Correct_speed = 0;
//	int16_t Inc_speed = 0;
//	//设置直线的角度目标
//	X_Correct_Position_PID.target = 0;
//	//等到扫描到二维码
//	while(QR_rx_flag!=1){
//		if(X_speed >= 0){
//			if(Inc_speed < Scan_Speed){Inc_speed += Speed_Acc;}
//			else{Inc_speed = Scan_Speed;}
//		}
//		else{
//			if(Inc_speed > -Scan_Speed){Inc_speed -= Speed_Acc;}
//			else{Inc_speed = -Scan_Speed;}
//		}
//		//解算修正值
//		X_Correct_Position_PID.measure = imu.yaw_set;
//		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
//		//设置修正值限幅
//		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
//		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
//		//控制电机解算并运行
//		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
//		//读取电机1的转过的角度
//		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
//		if(Motor1_inf.Rotted_Angle >= Motor_Angle*0.55){break;}
//	}

//	//加速运动
//	while(1){
//		if(X_speed >= 0){
//			if(Inc_speed < X_speed){Inc_speed += Speed_Acc;}
//			else{Inc_speed = X_speed;}
//		}
//		else{
//			if(Inc_speed > X_speed){Inc_speed -= Speed_Acc;}
//			else{Inc_speed = X_speed;}
//		}
//		//解算修正值
//		X_Correct_Position_PID.measure = imu.yaw_set;
//		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
//		//设置修正值限幅
//		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
//		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
//		//控制电机解算并运行
//		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
//		//读取电机1的转过的角度
//		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);			
//		//移动至路程的0.35时进行匀速
//		if(Inc_speed == X_speed){
//			break;
//		}
//	}
//	//匀速运动
//	while(1){
//		//解算修正值
//		X_Correct_Position_PID.measure = imu.yaw_set;
//		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
//		//设置修正值限幅
//		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
//		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
//		//控制电机解算并运行
//		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
//		//读取电机1的转过的角度
//		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);			
//		//移动至路程的0.8时进行减速
//		if(Motor1_inf.Rotted_Angle >= Decelerate*Motor_Angle){break;}
//	}
//	while(1){
//		if(X_speed >= 0){
//			if(Inc_speed > Brake_Speed){Inc_speed -= Speed_Dec;}
//			else{Inc_speed = Brake_Speed;}
//		}
//		else{
//			if(Inc_speed < -Brake_Speed){Inc_speed += Speed_Dec;}
//			else{Inc_speed = -Brake_Speed;}
//		}
//		//解算修正值
//		X_Correct_Position_PID.measure = imu.yaw_set;
//		Correct_speed=(int16_t)Position_PID(&X_Correct_Position_PID,&X_Correct_Param_PID);
//		//设置修正值限幅
//		if(Correct_speed >=  Correct_Speed_MAX)Correct_speed =  Correct_Speed_MAX;
//		if(Correct_speed <= -Correct_Speed_MAX)Correct_speed = -Correct_Speed_MAX;
//		//控制电机解算并运行
//		Motor_mecanum_wheel_speed_run(Inc_speed,0,Correct_speed,acc);
//		//读取电机1的转过的角度
//		Get_Motor_Rotted_Angle_Fun(&Motor_1,&Motor1_inf);
//		if(Motor1_inf.Rotted_Angle >= Motor_Angle){
//			Motor_mecanum_wheel_speed_run(0,0,0,acc);
//			break;			
//		}
//	}
//	Set_Motors_Zero(&Motor_1);
//}
