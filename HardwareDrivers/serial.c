#include "serial.h"

void Serial_Init(void)
{
	HAL_UART_Receive_IT(&huart1,&aRxBuffer,1);
}

void Serial_Send(UART_HandleTypeDef *huart,uint8_t *cmd)
{
	uint8_t size_cnt = strlen((const char *)cmd);
	HAL_UART_Transmit(huart,cmd,size_cnt,0xffff);
}

//定义在stm32f7xx_hal_uart.c里面的weak函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint32_t state = taskENTER_CRITICAL_FROM_ISR();
	if(huart->Instance == USART1)
	{
//		usmart_usartit();
	}
	else if(huart->Instance == USART2)
	{
		//IMU_UART
		//idle in stm32f7xx_it.c
	}
	else if(huart->Instance == UART4)
	{
		QR_UART_RxCpltCallback(huart);
	}
	else if(huart->Instance == UART5)
	{
		//STEPMOTOR_UART
		//idle in stm32f7xx_it.c
	}
	else if(huart->Instance ==USART6)
	{
		
	}
	else if(huart->Instance == UART7)
	{
		//OPENMV_UART
		//idle in stm32f7xx_it.c
		ARM_UART_RxCpltCallback(huart);
	}
	else if(huart->Instance == UART8)
	{
		RFID_UART_RxCpltCallback(huart);
	}
	taskEXIT_CRITICAL_FROM_ISR(state);
}
