#include "buzzer.h"
//PD7

//buzzer_task任务句柄
TaskHandle_t buzzer_task_handler;
//buzzer_task任务堆栈大小
#define buzzer_task_Size 32
//buzzer_task任务优先级
#define buzzer_task_Priority 2

uint8_t buzzer_wrong_flag = 0;
uint8_t buzzer_complete_flag = 0;

void Buzzer_three_times(void){
	uint8_t b_i =0;
	while(b_i<=2){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
		vTaskDelay(120);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
		vTaskDelay(120);
		b_i++;
	}
}

void Buzzer_two_times(void){
	uint8_t b_j =0;
	while(b_j<=1){
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
		vTaskDelay(80);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
		vTaskDelay(80);
		b_j++;
	}
}

void buzzer_task(void *pvParameters){
	while(1){
		vTaskDelay(1000);
		if(buzzer_wrong_flag == 1){
			Buzzer_three_times();
			buzzer_wrong_flag = 0;//重置
		}
		if(buzzer_complete_flag == 1){
			Buzzer_two_times();
			buzzer_complete_flag = 0;
		}
	}
	vTaskDelete(buzzer_task_handler);
}

void buzzer_task_create(void){
	xTaskCreate((TaskFunction_t ) 	buzzer_task,			//任务函数
				(const char *)		"buzzer_task",	  		//任务名字
				(uint32_t) 			buzzer_task_Size,    	//任务堆栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		buzzer_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&buzzer_task_handler 	//任务句柄
				);		
}