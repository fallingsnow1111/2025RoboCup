#include "delay.h"

void delay_init(void){
	HAL_TIM_Base_Start_IT(&htim3);//使能TIM3
}

//最多延时5000us
void delay_us(uint16_t nus)
{
	TIM7->CNT = 0;
	while(TIM7->CNT<nus);
}


void delay_ms(uint16_t nms)
{
	for(uint16_t i=0;i<nms;i++){
		delay_us(1000);
	}
}

