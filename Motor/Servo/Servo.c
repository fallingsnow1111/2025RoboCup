#include "Servo.h"

void servo_init(void){
	USART6->ICR = USART6->ICR | (0x01<<4);
}

void servo_control(uint8_t servo_cmd_id ){
	uint8_t cmd_arr[7]={0};
	cmd_arr[0] = 0x55;
	cmd_arr[1] = 0x55;
	cmd_arr[2] = 0x05;
	cmd_arr[3] = 0x06;			//运行动作组
	cmd_arr[4] = servo_cmd_id;	//要运行的动作组编号
	cmd_arr[5] = 0x01;			//运行次数低八位
	cmd_arr[6] = 0x00;			//运行次数高八位
	HAL_UART_Transmit(&huart7,cmd_arr,sizeof(cmd_arr),0xffff);
	if(servo_cmd_id != 0x00)
		vTaskDelay(1500);
}




