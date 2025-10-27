#ifndef __MOTOR_COMMAND_H
#define __MOTOR_COMMAND_H

#include "delay.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
//#include "uart.h"
#include "gpio.h"
#include "buzzer.h"
#include "Motor_Run.h"
#include <string.h>

#define Cmd_Comfirm_BUFFER_SIZE 10
#define Cmd_Return_BUFFER_SIZE 16
#define Motor_Run_DelayTime 3 //仅发送运动和触发指令加Cmd_Comfirm的时间
#define Motor_Get_DelayTime 5 //仅发送读取参数指令加Cmd_Comfirm的时间



extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
//extern DMA_HandleTypeDef hdma_uart7_rx;
//extern DMA_HandleTypeDef hdma_uart7_tx;
extern UART_HandleTypeDef huart4;
//xtern TIM_HandleTypeDef htim6;

extern uint8_t Cmd_Comfirm_rx_buf[Cmd_Comfirm_BUFFER_SIZE];//接收指令是否发送成功并正确
extern uint8_t Cmd_Comfirm_rx_len;
extern uint8_t Cmd_correct_flag;//是否发送成功并正确标志位

extern uint8_t Cmd_Rrturn_rx_buf[Cmd_Return_BUFFER_SIZE];
extern uint8_t Cmd_Rrturn_rx_len;
extern uint8_t Cmd_Return_flag;//是否发送成功并正确标志位


#define motor_handler huart3//步进电机串口句柄
//USART_TX PB10
//USART_RX PB11

void motor_usart_init(void);
void motor_usart_start_receive(uint8_t receive_mode_flag);
void Clr_Cmd_Rrturn_rx_buf(void);

struct Stepping_Motor_Param
{
	uint8_t  Motor_Addr;			/*步进电机地址:可设置的地址为 1-247，地址 0 作为广播地址，上位机以 0
                                    地址下发命令后，所有连接到上位机的闭环电机都会执行这条命令*/
	
	uint8_t  Motor_Dir;      		/*步进电机旋转方向:可设置数据为 0或1*/
	
	uint16_t Motor_Speed;			/*步进电机的速度:可设置的速度档位为 0-1279*/
	
	uint8_t  Motor_Acc;				/*步进电机加速度:可设置的加速度档位为 0-255*/
	
	uint32_t Pulse_Number; 			/*脉冲数:可设置的脉冲数 0-16,777,215*/
	
	uint8_t  Motor_Enable;			/*步进电机使能状态: 0x00对应不使能；0x01对应使能*/
	
	uint8_t  Check_Byte;			/*校验字节:默认为 0x6B*/

};

struct Motor_Inf
{
	uint32_t  Motor_Position;		/*电机位置：范围：-2147483647 ~ 2147483647*/
	
	int16_t  Motor_Position_Error;	/*电机位置误差：范围：-32767 ~ 32767*/
	
	uint16_t Motor_Coder;			/*编码器数值：0 ~ 65535*/
	
	float  	 Rotted_Angle;      	/*转过的角度：范围：-32767 ~ 32767 =( Motor_Position * 360 )/65536 */
	
	float	 Angle_Error;			/*角度误差：范围：-359 ~ 359 =( Motor_Position_Error * 360 )/65536*/
	
	float 	 Angle;					/*角度：范围：0 ~ 360° Angle = Motor_Coder/65536 */
	
	int32_t  Input_Pulse_Number;	/*实际输入脉冲数：范围：-2147483647 ~ 2147483647 正转输入脉冲为正	*/
	
	uint8_t  Motor_Lock;			/*堵转标志 0x00表示没有堵转 0x01表示堵转*/
	
};

extern struct Stepping_Motor_Param Motor_1;
extern struct Stepping_Motor_Param Motor_2;
extern struct Stepping_Motor_Param Motor_3;
extern struct Stepping_Motor_Param Motor_4;

extern struct Motor_Inf Motor1_inf;
extern struct Motor_Inf Motor2_inf;
extern struct Motor_Inf Motor3_inf;
extern struct Motor_Inf Motor4_inf;

//获取转过角度命令
float Get_Motor_Rotted_Angle_Fun(struct Stepping_Motor_Param* Motor,struct Motor_Inf* Motor_inf);//转过的角度=（读取值*360）/65536
float Get_Motor_Rotted_Angle_Fun_have_negative(struct Stepping_Motor_Param* Motor,struct Motor_Inf* Motor_inf);//转过的角度=（读取值*360）/65536
//触发动作命令
void Set_Motors_Zero(struct Stepping_Motor_Param*motor);
//运动控制命令
int16_t Mecanum_Wheel_Solve(struct Stepping_Motor_Param* motor,int16_t X,int16_t Y,int16_t Omg);
void Motor_Speed_Control(struct Stepping_Motor_Param* Motor);
void Motor_Speed_Pulse_Control(struct Stepping_Motor_Param Motor);
void Motor_Run(void);


#endif
