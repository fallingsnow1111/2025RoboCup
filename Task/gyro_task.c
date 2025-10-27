#include "gyro_task.h"

//gyro_task任务句柄
TaskHandle_t gyro_task_handler;
//gyro_task任务堆栈大小
#define gyro_task_Size 64
//gyro_task任务优先级
#define gyro_task_Priority 14//与main_task优先级相同

//x_delay_task任务句柄
TaskHandle_t x_delay_task_handler;
//x_delay_task任务堆栈大小
#define x_delay_task_Size 64
//x_delay_task任务优先级
#define x_delay_task_Priority 14//与main_task优先级相同

//y_delay_task任务句柄
TaskHandle_t y_delay_task_handler;
//y_delay_task任务堆栈大小
#define y_delay_task_Size 64
//y_delay_task任务优先级
#define y_delay_task_Priority 14//与main_task优先级相同

uint8_t stop_flag = 0;//旋转校正停止标志
uint8_t x_stop_flag = 0;//x修正停止标志
uint8_t y_stop_flag = 0;//y修正停止标志
float yaw_set_arv =0;


void gyro_task(void *pvParameters){
		vTaskDelay(50);
		stop_flag = 1;
	vTaskDelete(gyro_task_handler);
}

void x_delay_task(void *pvParameters){
		vTaskDelay(400);
		x_stop_flag = 1;
	vTaskDelete(x_delay_task_handler);
}

void y_delay_task(void *pvParameters){
		vTaskDelay(400);
		y_stop_flag = 1;
	vTaskDelete(y_delay_task_handler);
}

void gyro_task_create(void){
	xTaskCreate((TaskFunction_t ) 	gyro_task,			//任务函数
				(const char *)		"gyro_task",	  	//任务名字
				(uint32_t) 			gyro_task_Size,    	//任务堆栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		gyro_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&gyro_task_handler 	//任务句柄
				);
}

void x_delay_task_create(void){
	xTaskCreate((TaskFunction_t ) 	x_delay_task,			//任务函数
				(const char *)		"x_delay_task",	  	//任务名字
				(uint32_t) 			x_delay_task_Size,    	//任务堆栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		x_delay_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&x_delay_task_handler 	//任务句柄
				);
}

void y_delay_task_create(void){
	xTaskCreate((TaskFunction_t ) 	y_delay_task,			//任务函数
				(const char *)		"y_delay_task",	  	//任务名字
				(uint32_t) 			y_delay_task_Size,    	//任务堆栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		y_delay_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&y_delay_task_handler 	//任务句柄
				);
}