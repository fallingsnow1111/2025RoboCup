#include "led.h"

//led_task任务句柄
TaskHandle_t led_task_handler;
//led_task任务堆栈大小
#define led_task_Size 32
//led_task任务优先级
#define led_task_Priority 1

void TogglePin_PB9(){
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
}

void led_task(void *pvParameters){
	while(1){
		vTaskDelay(300);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
	}
	vTaskDelete(led_task_handler); 
}

void led_task_create(void){
	xTaskCreate((TaskFunction_t ) 	led_task,			//任务函数
				(const char *)		"led_task",	  		//任务名字
				(uint32_t) 			led_task_Size,    	//任务堆栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		led_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&led_task_handler 	//任务句柄
				);		
}

