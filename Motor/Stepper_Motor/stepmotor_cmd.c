#include "stepmotor_cmd.h"
#include "stdio.h"
uint8_t stpmotRxbuff[21] = {0};
uint8_t stpmotRecFlag = 0;
volatile uint8_t stpmot_spin_sta = 0;	//1 电机在转 0 电机停止

void stpmot_init(void)
{
	__HAL_UART_ENABLE_IT(&STEPMOTOR_UART, UART_IT_IDLE);	//空闲中断
	HAL_UART_Receive_DMA(&STEPMOTOR_UART,stpmotRxbuff,20);
//	HAL_UART_Receive_IT(&STEPMOTOR_UART,stpmotRxbuff,4);
}

void STPMOT_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
		if(RESET != __HAL_UART_GET_FLAG(&STEPMOTOR_UART, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&STEPMOTOR_UART);                     //清楚空闲中断标志（否则会一直不断进入中断）
            STPMOT_UART_IDLECallback(huart);                          //调用中断处理函数
        }
}

void STPMOT_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(&STEPMOTOR_UART);     
//	HAL_UART_Transmit(&huart1,stpmotRxbuff,4,0xffff);
	//停止本次DMA传输
	if((stpmotRxbuff[0] == 0x01)&&
		(stpmotRxbuff[1]== 0xfd)&&
		(stpmotRxbuff[2] == 0x9f)&&
		(stpmotRxbuff[3] == 0x6b))
	{
		stpmot_spin_sta = 0;
		//printf("stpmot_revced!\n");
	}
    HAL_UART_Receive_DMA(&STEPMOTOR_UART,stpmotRxbuff,20);
}
/*
void STEPMOTOR_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if((stpmotRxbuff[0] == 0x01)&&
		(stpmotRxbuff[1]== 0xfd)&&
		(stpmotRxbuff[2] == 0x9f)&&
		(stpmotRxbuff[3] == 0x6b))
	{
		stpmot_spin_sta = 0;
	}
	else
	{
		for(uint8_t i = 0;i < 4;i++)
		{
			stpmotRxbuff[i] = 0;
		}
	}
	HAL_UART_Receive_IT(&STEPMOTOR_UART,stpmotRxbuff,4);
}
*/
const uint8_t acc_inc = 128;
const uint16_t rpm_speed = 3000;
uint8_t dir = 0x01;			//0x00 ccw	0x01 cw

void stpmot_to_degree(uint32_t degree,uint8_t direct)
{
	uint8_t sendbuff[13] = {0};
	sendbuff[0] = 0x01;
	sendbuff[1] = 0xfd;
	sendbuff[2] = direct;
	sendbuff[3] = rpm_speed>>8;
	sendbuff[4] = rpm_speed;

	sendbuff[5] = acc_inc;

	sendbuff[6] = degree>>24;
	sendbuff[7] = degree>>16;
	sendbuff[8] = degree>>8;
	sendbuff[9] = degree;
	
	sendbuff[10] = 0x00;	//00 相对	01绝对
	sendbuff[11] = 0x00;
	sendbuff[12] = 0x6b;
	
	HAL_UART_Transmit(&STEPMOTOR_UART,sendbuff,13,0xffff);
	stpmot_spin_sta = 1;
}

uint8_t stpmot_work_sta(void)
{
	if(stpmot_spin_sta)return 1;
	else return 0;
}

void stpmot_enable(uint8_t sta)	//0x01 en  0x00 dis
{
	//默认已使能
	uint8_t sendbuff[6] = {0};
	sendbuff[0] = 0x01;
	sendbuff[0] = 0xf3;
	sendbuff[0] = 0xab;
	sendbuff[0] = sta;
	sendbuff[0] = 0;
	sendbuff[0] = 0x6b;
	
	HAL_UART_Transmit(&STEPMOTOR_UART,sendbuff,6,0xffff);
	//succ return 01 f3 02 6b
}
uint8_t find_zero_sta = 0;
void stpmot_find_zero(void)
{
	uint8_t sendbuff[5] = {0};
	sendbuff[0] = 0x01;
	sendbuff[1] = 0x9a;
	sendbuff[2] = 0x03;		//00表示触发单圈就近回零，01 表示触发单圈方向回零，02 表示触发多圈无限位碰撞回零，03 表示触发多圈有限位开关回零
	sendbuff[3] = 0x00;
	sendbuff[4] = 0x6b;
	
	HAL_UART_Transmit(&STEPMOTOR_UART,sendbuff,5,0xffff);
	find_zero_sta = 1;
}

uint8_t stpmot_find_zero_sta(void)
{
	return find_zero_sta;
}

void stpmot_stop_find_zero(void)
{
	uint8_t sendbuff[4] = {0};
	sendbuff[0] = 0x01;
	sendbuff[1] = 0x9c;
	sendbuff[2] = 0x48;
	sendbuff[3] = 0x6b;
	HAL_UART_Transmit(&STEPMOTOR_UART,sendbuff,4,0xffff);
	find_zero_sta = 0;
}
