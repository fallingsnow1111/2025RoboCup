#include "jetson.h"
#include "main_task.h"
#include "stdbool.h"
//*****  PC6
//*****  PC7   USART6


//设置画面中心点 摄像头为
//向左、向上为正（左小右大）

#define Jetson_handle huart6
#define Jetson_BUFFER_SIZE 5

uint8_t Jetson_rx_buf[Jetson_BUFFER_SIZE];
uint8_t Jetson_rx_len = 0;
uint8_t Jetson_mode_flag = 0;
//以下定义用于模式切换发送
uint8_t RED_Mode[] = {1};
uint8_t BLUE_Mode[] = {2};
uint8_t claw_loose[] = {3};
uint8_t claw_catch[] = {4};

_Bool catch_flag = false;
_Bool loose_flag = false;


void jetson_rx_init(void){
	USART6->ICR = USART6->ICR | (0x01<<4);
	__HAL_UART_ENABLE_IT(&Jetson_handle, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&Jetson_handle,Jetson_rx_buf,Jetson_BUFFER_SIZE);
}


void RED_Mode_Change(void){
	HAL_UART_Transmit(&Jetson_handle,RED_Mode,sizeof(RED_Mode),0xffff);
}
void BLUE_Mode_Change(void){
	HAL_UART_Transmit(&Jetson_handle,BLUE_Mode,sizeof(BLUE_Mode),0xffff);
}
void Claw_catch(void){
	HAL_UART_Transmit(&Jetson_handle,claw_catch,sizeof(claw_catch),0xffff);
}
void Claw_loose(void){
	HAL_UART_Transmit(&Jetson_handle,claw_loose,sizeof(claw_loose),0xffff);
}


void USART6_IRQHandler(void)
{
	uint32_t Jetson_tmp_flag = 0;
	uint32_t Jetson_temp;
	Jetson_tmp_flag =__HAL_UART_GET_FLAG(&Jetson_handle,UART_FLAG_IDLE); //获取IDLE标志位
	
	if((Jetson_tmp_flag != RESET))//idle标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&Jetson_handle);//清除标志位
		HAL_UART_DMAStop(&Jetson_handle); 
		Jetson_temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 获取DMA中未传输的数据个数   
		Jetson_rx_len =  Jetson_BUFFER_SIZE - Jetson_temp; //总计数减去未传输的数据个数，得到已经接收的数据个数

		//处理数据部分
		if(Jetson_rx_buf[Jetson_rx_len-1] == 0xAA){
			memset(Jetson_rx_buf,0,Jetson_BUFFER_SIZE);
			catch_flag = true;
		}
		else if(Jetson_rx_buf[Jetson_rx_len-1] == 0x0A){
			memset(Jetson_rx_buf,0,Jetson_BUFFER_SIZE);
			loose_flag = true;
		}
	 }
	//清空Jetson_rx_buf和Jetson_rx_len
	memset(Jetson_rx_buf,0,Jetson_BUFFER_SIZE);
	Jetson_rx_len = 0;

	HAL_UART_Receive_DMA(&Jetson_handle,Jetson_rx_buf,Jetson_BUFFER_SIZE);	
	HAL_UART_IRQHandler(&Jetson_handle);
}
