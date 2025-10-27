#include "Motor_Command.h"
#define Motor_Run_DelayTime 3 //仅发送运动和触发指令加Cmd_Comfirm的时间
#define Motor_Get_DelayTime 5 //仅发送读取参数指令加Cmd_Comfirm的时间
#define Cmd_Comfirm_BUFFER_SIZE 10
#define Cmd_Return_BUFFER_SIZE 16


//float x_dis  = 0;
//float y_dis  = 0;
//float w_real = 0;


/**************************/
/*		底盘步进设定		  */
/*		TX —— PB10		  */
/*		RX —— PB11		  */
/**************************/
#define motor_handler huart3//步进电机串口句柄

struct Stepping_Motor_Param Motor_1=
	{	
		.Motor_Addr 	= 0x01,//不可改动
		.Motor_Dir 		= 0x10>>4,
		.Motor_Speed 	= 0x01<<8  | 0x00,
		.Motor_Acc 		= 0x20,
		.Pulse_Number 	= 0x00<<24 | 0x00<<16 | 0x7D<<8 | 0x00,
		.Motor_Enable 	= 0x01,
		.Check_Byte 	= 0x6B,//不可改动
	};
struct Stepping_Motor_Param Motor_2=
	{	
		.Motor_Addr 	= 0x02,//不可改动
		.Motor_Dir 		= 0x10>>4,
		.Motor_Speed 	= 0x01<<8  | 0x00,
		.Motor_Acc 		= 0x20,
		.Pulse_Number 	= 0x00<<24 | 0x00<<16 | 0x7D<<8 | 0x00,
		.Motor_Enable 	= 0x01,
		.Check_Byte 	= 0x6B,//不可改动
	};
struct Stepping_Motor_Param Motor_3=
	{	
		.Motor_Addr 	= 0x03,//不可改动
		.Motor_Dir 		= 0x10>>4,
		.Motor_Speed 	= 0x01<<8  | 0x00,
		.Motor_Acc 		= 0x20,
		.Pulse_Number 	= 0x00<<24 | 0x00<<16 | 0x7D<<8 | 0x00,
		.Motor_Enable 	= 0x01,
		.Check_Byte 	= 0x6B,//不可改动
	};
struct Stepping_Motor_Param Motor_4=
	{	
		.Motor_Addr 	= 0x04,//不可改动
		.Motor_Dir 		= 0x10>>4,
		.Motor_Speed 	= 0x01<<8  | 0x00,
		.Motor_Acc 		= 0x20,
		.Pulse_Number 	= 0x00<<24 | 0x00<<16 | 0x7D<<8 | 0x00,
		.Motor_Enable 	= 0x01,
		.Check_Byte 	= 0x6B,//不可改动
	};

struct Motor_Inf Motor1_inf = {0};
struct Motor_Inf Motor2_inf = {0}; 
struct Motor_Inf Motor3_inf = {0};
struct Motor_Inf Motor4_inf = {0};

uint8_t rx_end_flag = 0;
uint8_t receive_mode_flag = 0;
//receive_mode_flag	置0时	接收触发动作命令、运动控制命令是否正确；
//receive_mode_flag	置1时	接收读取参数命令信息

/*******************触发动作命令、运动控制命令*******************/
uint8_t Correct[4]	=	{0x00,0x00,0x02,0x6B};	//命令正确 地址 + 发送的命令 + 02 + 校验字节
uint8_t Error[4]	=	{0x00,0x00,0xE2,0x6B};	//命令错误 地址 + 发送的命令 + EE + 校验字节
uint8_t Cmd_Comfirm_rx_buf[Cmd_Comfirm_BUFFER_SIZE]={0};//接收指令是否发送成功并正确
uint8_t Cmd_Comfirm_rx_len = 0;
uint8_t Cmd_correct_flag = 0;//是否发送成功并正确标志位

/**********************接收读取参数命令信息**********************/
uint8_t Cmd_Return_rx_buf[Cmd_Return_BUFFER_SIZE]={0};
uint8_t Cmd_Return_rx_len = 0;
uint8_t Cmd_Return_flag = 0;//是否发送成功并正确标志位


uint8_t cmd3[3]={0};//读取参数命令 
uint8_t cmd4[4]={0};//触发动作命令
uint8_t cmd8[8]={0};//速度控制
uint8_t cmd13[13]={0};//速度位置控制


void motor_usart_init(void){
	USART3->ICR = USART3->ICR | (0x01<<4);
	__HAL_UART_ENABLE_IT(&motor_handler,UART_IT_IDLE); //使能IDLE中断
}


void motor_usart_start_receive(uint8_t receive_mode_flag){
	if(receive_mode_flag == 0){
		__HAL_UART_ENABLE_IT(&motor_handler,UART_IT_IDLE); //使能IDLE中断
		HAL_UART_Receive_DMA(&motor_handler,Cmd_Comfirm_rx_buf,Cmd_Comfirm_BUFFER_SIZE);//接收触发动作命令、运动控制命令是否正确
	}
	if(receive_mode_flag == 1){
		__HAL_UART_ENABLE_IT(&motor_handler,UART_IT_IDLE); //使能IDLE中断
		HAL_UART_Receive_DMA(&motor_handler,Cmd_Return_rx_buf,Cmd_Return_BUFFER_SIZE);//接收读取参数命令信息
	}
}

void Clr_Cmd_Return_rx_buf(void){
	memset(Cmd_Return_rx_buf,0,Cmd_Return_BUFFER_SIZE);//清接收缓存
	Cmd_Return_rx_len = 0;	
}


int16_t Mecanum_Wheel_Solve(struct Stepping_Motor_Param* motor,int16_t X,int16_t Y,int16_t Omg){
	int16_t speed = 0;
	switch(motor->Motor_Addr){
		case 0x01:	
			speed = +Y - X + Omg*0.5;
			break;
		case 0x02:
			speed = +Y + X + Omg*0.5;
			break;
		case 0x03:
			speed = -Y - X + Omg*0.5;
			break;
		case 0x04:
			speed = -Y + X + Omg*0.5;
			break;
	}
	
	if(speed>=15000)speed=15000;if(speed<=-15000)speed=-15000;	
	return speed;
}



float Get_Motor_Rotted_Angle_Fun(struct Stepping_Motor_Param* Motor,struct Motor_Inf* Motor_inf){
	//发送指令
	cmd3[0]=Motor->Motor_Addr;  //地址
	cmd3[1]=0x36;//该项不可更改  //读取电机实时位置的一个帧头
	cmd3[2]=Motor->Check_Byte;  //校验字节
	receive_mode_flag = 1;
	while(Cmd_Return_flag != 1)
	{
		Clr_Cmd_Return_rx_buf();//清空接收缓存
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd3,sizeof(cmd3),0xffff) != HAL_OK)//DMA方式发送指令
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Get_DelayTime);
	}	
	//对进入中断后接收到的信息进行处理
	Motor_inf->Motor_Position = (   (Cmd_Return_rx_buf[3]<<24)|
									(Cmd_Return_rx_buf[4]<<16)|
									(Cmd_Return_rx_buf[5]<<8 )|
									(Cmd_Return_rx_buf[6]<<0 )
								);
	Motor_inf->Rotted_Angle =( Motor_inf->Motor_Position * 360 )/65536;
	//处理结束
	Clr_Cmd_Return_rx_buf();//清空接收缓存
	Cmd_Return_flag = 0;//指令接收标志位置位
	
	return Motor_inf->Rotted_Angle;
}

float Get_Motor_Rotted_Angle_Fun_have_negative(struct Stepping_Motor_Param* Motor,struct Motor_Inf* Motor_inf){
	//发送指令
	cmd3[0]=Motor->Motor_Addr;  //地址
	cmd3[1]=0x36;//该项不可更改  //读取电机实时位置的一个帧头
	cmd3[2]=Motor->Check_Byte;  //校验字节
	receive_mode_flag = 1;
	while(Cmd_Return_flag != 1)
	{
		Clr_Cmd_Return_rx_buf();//清空接收缓存
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd3,sizeof(cmd3),0xffff) != HAL_OK)//DMA方式发送指令
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Get_DelayTime);
	}	
	//对进入中断后接收到的信息进行处理
	Motor_inf->Motor_Position = (   (Cmd_Return_rx_buf[3]<<24)|
									(Cmd_Return_rx_buf[4]<<16)|
									(Cmd_Return_rx_buf[5]<<8 )|
									(Cmd_Return_rx_buf[6]<<0 )
								);
	Motor_inf->Rotted_Angle =( Motor_inf->Motor_Position * 360 )/65536;
	//正负判断
	if(Cmd_Return_rx_buf[2] == 0x01) Motor_inf->Rotted_Angle = - Motor_inf->Rotted_Angle;
	else if(Cmd_Return_rx_buf[2] == 0x00) Motor_inf->Rotted_Angle = Motor_inf->Rotted_Angle;
		
	//处理结束
//	Clr_Cmd_Return_rx_buf();//清空接收缓存
	Cmd_Return_flag = 0;//指令接收标志位置位
	return Motor_inf->Rotted_Angle;
}

//编码器清零
void Set_Motors_Zero(struct Stepping_Motor_Param*motor){
	//发送指令
	cmd4[0]=motor->Motor_Addr;//该项不可更改
	cmd4[1]=0x0A;//该项不可更改
	cmd4[2]=0x6D;//该项不可更改
	cmd4[3]=0x6B;//该项不可更改
	receive_mode_flag = 0;
	while(Cmd_correct_flag != 1)
	{
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd4,sizeof(cmd4),0xffff) != HAL_OK)//DMA方式发送指令
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Run_DelayTime);
	}	
	Cmd_correct_flag = 0;//指令接收标志位置位
}

void Motor_Run(void){
	//发送指令
	cmd4[0]=0x00;//该项不可更改
	cmd4[1]=0xFF;//该项不可更改
	cmd4[2]=0x66;//该项不可更改
	cmd4[3]=0x6B;//该项不可更改
	receive_mode_flag = 0;
	while(Cmd_correct_flag != 1)
	{
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd4,sizeof(cmd4),0xffff) != HAL_OK)//DMA方式发送指令
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Run_DelayTime);
	}	
	Cmd_correct_flag = 0;//指令接收标志位置位
}

//运动控制命令   //速度模式控制
void Motor_Speed_Control(struct Stepping_Motor_Param* Motor){
	cmd8[0] = Motor->Motor_Addr;				/* 电机地址 */
	cmd8[1] = 0xF6;//该项不可更改				/* 功能码 */
	cmd8[2] = Motor->Motor_Dir;					/* 方向字节 */
	cmd8[3] = Motor->Motor_Speed >> 8;			/* 速度高字节 */
	cmd8[4] = Motor->Motor_Speed >> 0;			/* 速度低字节 */
	cmd8[5] = Motor->Motor_Acc;					/* 加速度 */
	cmd8[6] = 0x01;//0x01表示启用同步控制		/* 同步控制位 */
	cmd8[7] = Motor->Check_Byte;				/* 固定校验字节 */
	receive_mode_flag = 0;
	while(Cmd_correct_flag != 1){
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd8,sizeof(cmd8),0xffff) != HAL_OK)
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Run_DelayTime);
	}
	Cmd_correct_flag = 0;//指令接收标志位置位
}

void Motor_Speed_Pulse_Control(struct Stepping_Motor_Param Motor){
	cmd13[0]  = Motor.Motor_Addr;				/* 电机地址 */
	cmd13[1]  = 0xFD;//该项不可更改				/* 功能码 */
	cmd13[2]  = Motor.Motor_Dir;				/* 方向字节 */
	cmd13[3]  = Motor.Motor_Speed >> 8;			/* 速度高字节 */
	cmd13[4]  = Motor.Motor_Speed >> 0;			/* 速度低字节 */
	cmd13[5]  = Motor.Motor_Acc;				/* 加速度 */
	cmd13[6]  = Motor.Pulse_Number >> 24;		/* 脉冲数0字节 */
	cmd13[7]  = Motor.Pulse_Number >> 16;		/* 脉冲数1字节 */
	cmd13[8]  = Motor.Pulse_Number >> 8;		/* 脉冲数2字节 */
	cmd13[9]  = Motor.Pulse_Number >> 0;		/* 固定校3字节 */
	cmd13[10] = 0x01;//0x01表示启用绝对位置模式	/* 相对、绝对运动字节 */
	cmd13[11] = 0x01;//0x01表示启用同步控制		/* 同步控制字节 */
	cmd13[12] = Motor.Check_Byte;				/* 速度字节 */
	receive_mode_flag = 0;
	while(Cmd_correct_flag != 1){
		motor_usart_start_receive(receive_mode_flag);
		if(HAL_UART_Transmit(&motor_handler,(uint8_t*)cmd13,sizeof(cmd13),0xffff) != HAL_OK)//DMA方式发送指令
		{
			Error_Handler();
		}
		vTaskDelay(Motor_Run_DelayTime);
	}
	Cmd_correct_flag = 0;//指令接收标志位置位
}

//每次生成cubmx都要注释掉默认的串口中断函数，否则重定义
void USART3_IRQHandler(void)
{
	uint32_t tmp_flag = 0;
	uint32_t temp = 0;
	HAL_UART_IRQHandler(&motor_handler);
	tmp_flag =__HAL_UART_GET_FLAG(&motor_handler,UART_FLAG_IDLE); //获取IDLE标志位
	
	/******************************************接收模式1******************************************/
	/***************************接收触发动作命令、运动控制命令是否正确*****************************/
	
	if(receive_mode_flag == 0){//未接收到指令正确发送信息
		if((tmp_flag != RESET)){//IDLE标志被置位 
			
			__HAL_UART_CLEAR_IDLEFLAG(&motor_handler);//清除IDLE标志位
			HAL_UART_DMAStop(&motor_handler); //停止DMA接收
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数   
			Cmd_Comfirm_rx_len =  Cmd_Comfirm_BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
			rx_end_flag = 1;//接收完成标志位置1
			
			//判断接收完成且为非空信息
			if(rx_end_flag == 1 && Cmd_Comfirm_rx_len != 0){
				//判断指令是否发送成功并正确
				if(Cmd_Comfirm_rx_buf[Cmd_Comfirm_rx_len - 2] == Correct[2] && Cmd_Comfirm_rx_buf[Cmd_Comfirm_rx_len - 1] == Correct[3]){
					Cmd_correct_flag = 1;
				}
				else{
					Cmd_correct_flag = 0;
				}
			}
			
			//清空Cmd_Comfirm_rx_buf和Cmd_Comfirm_rx_len
			memset(Cmd_Comfirm_rx_buf,0,Cmd_Comfirm_BUFFER_SIZE);//清接收缓存
			Cmd_Comfirm_rx_len = 0;	
		}
	}
	/********************************************************************************************/
	
	
	/******************************************接收模式2******************************************/
	/*************************************接收读取参数命令信息*************************************/
	
	if(receive_mode_flag == 1){//接收到指令正确发送信息
		if((tmp_flag != RESET)){//IDLE标志被置位
		
			__HAL_UART_CLEAR_IDLEFLAG(&motor_handler);//清除IDLE标志位
			HAL_UART_DMAStop(&motor_handler); //停止DMA接收
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);// 获取DMA中未传输的数据个数
			Cmd_Return_rx_len =  Cmd_Return_BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
			rx_end_flag = 1;//接收完成标志位置1
			
			//判断接收完成且为非空信息
			if(rx_end_flag == 1 && Cmd_Return_rx_len != 0){
					if(Cmd_Return_rx_buf [Cmd_Return_rx_len-1] == 0x6B){
						Cmd_Return_flag = 1;
					}
				}
				else{
					Cmd_Return_flag = 0;
				}
			//清空接收缓存部分在主函数进行
			}
		}
	/********************************************************************************************/
	rx_end_flag = 0;//清除接收完成标志位
	HAL_UART_IRQHandler(&motor_handler);
}


