#ifndef __MOTOR_RUN_H
#define __MOTOR_RUN_H

#include "Motor_Command.h"
#include "pid.h"
#include "imu.h"
#include "Servo.h"
#include "gyro_task.h"
#include "main_task.h"
#include "math.h"
//正解算参数
#define 	kx 			1.457
#define 	ky 			1.463
#define 	kw 			-6.053
#define     kd			-13.813-14.973


//用于改变电机结构体信息
void Stepping_Motor_Param_change(	struct Stepping_Motor_Param* Motor_Param,
									uint32_t Pluse_num,
									uint8_t dir,
									uint16_t speed,
									uint8_t acc
									);


uint32_t Distance_to_Pluse_Num(float dis);
uint32_t Angle_to_Pluse_Num(float agl);							
//电机逆时针 Motor_Dir=1 角度变换为正
//电机顺时针 Motor_Dir=0 角度变换为负
//小车三种行动模式 直线、旋转、平移									
/***************************************************************************
straight_run		  向前							     向后
									
					   头							      头
			
			  逆//1			2\\顺				顺//1			2\\逆
		
		
		
			
			  逆\\3			4//顺				顺\\3			4//逆

									
可控变量：	1、距离（distance）	单位mm
			2、方向（dir）		0表示向后，1表示向前
			2、速度（speed）		0-1279(0x04 0xFF)
			4、加速度（acc）		0-255(FF)
***************************************************************************/
void Motor_straight_run(float distance,uint8_t dir,uint16_t speed,uint8_t acc);


/***************************************************************************
translate_run		  向左							     向右
									
					   头							      头
			
			  顺//1			2\\顺				逆//1			2\\逆
		
		
		
			
			  逆\\3			4//逆				顺\\3			4//顺

									
可控变量：	1、距离（distance）	单位mm
			2、方向（dir）		0表示向左，1表示向右
			2、速度（speed）		0-1279(0x04 0xFF)
			4、加速度（acc）		0-255(FF)
***************************************************************************/
void Motor_translate_run(float distance,uint8_t dir,uint16_t speed,uint8_t acc);


/***************************************************************************
rot_run				 顺时针							    逆时针
									
					   头							      头
			
			  顺//1			2\\顺				逆//1			2\\逆
		
		
		
			
			  顺\\3			4//顺				逆\\3			4//逆

									
可控变量：	1、角度（angle）		单位(度°)
			2、方向（dir）		0表示顺时针转动 1表示逆时针转动
			3、速度（speed）		0-1279(0x04 0xFF)
			4、加速度（acc）		0-255(FF)
***************************************************************************/
void Motor_rot_run(float angle,uint8_t dir,uint16_t speed,uint8_t acc);


/***************************************************************************
麦克纳姆轮解算		    X
					   	|					      
						|
			  顺\\1		|	2//顺				
						|
						|
			Y——————
		
			
			  顺//3			4\\顺			
			  
			 垂直X轴指向Y轴为Omega
***************************************************************************/


//麦克纳姆轮速度解算、电机控制
void Motor_mecanum_wheel_speed_run(int16_t X_speed,int16_t Y_speed,int16_t Omega,uint8_t acc);
void Motor_mecanum_wheel_pid_straight_run(float Motor_Angle,int16_t X_speed,uint8_t acc);
void Motor_mecanum_wheel_pid_straight_run_smooth(float Motor_Angle,int16_t X_speed,uint8_t acc,uint8_t Speed_Acc,float Accelerate,uint8_t Speed_Dec,float Decelerate);
//void Motor_mecanum_wheel_pid_straight_run_smooth_scan(float Motor_Angle,int16_t X_speed,uint8_t acc,uint8_t Speed_Acc,float Accelerate,uint8_t Speed_Dec,float Decelerate);
void Motor_mecanum_wheel_pid_translate_run(float Motor_Angle,int16_t Y_speed,uint8_t acc);
void Motor_mecanum_wheel_pid_rot_run(float angle,uint8_t acc);
void Motor_mecanum_wheel_straight_pid_rot_run(float Motor_Angle,int16_t X_speed,float angle,uint8_t acc);
void Motor_mecanum_wheel_straight_pid_rot_run_new(int16_t X_dis,int16_t Y_dis,int16_t X_speed,int16_t Y_speed,float Omg,uint8_t acc);


void Motor_mecanum_wheel_advance_at_an_angle(float Motor_Angle,int16_t X_speed,int16_t Y_speed,uint8_t acc);
void Motor_mecanum_wheel_pid_oblique_run(float xdistance,float ydistance,int16_t oblique_speed,uint8_t acc);
void Motor_mecanum_wheel_pid_rot_run_muti_angle(float Motor_Angle);
void Motor_mecanum_wheel_pid_straight_run_by_real_distance(float real_distance_X,float real_distance_Y,int16_t X_speed,int16_t Y_speed,uint8_t acc);
//无解算,陀螺仪PID控制直行、平移
void Motor_gyro_straight_run(uint8_t dir,uint16_t speed,uint8_t acc);
void Motor_gyro_translate_run(uint8_t dir,uint16_t speed,uint8_t acc);


#endif

