#include "screen.h"

//start_task任务句柄
TaskHandle_t screen_task_handler;
//start_task任务堆栈大小
#define screen_task_Size 128
//start_task任务优先级
#define screen_task_Priority 14

//VCC	->	VCC
//GND	->	GND
//PA4  	->	SCL
//PC4  	->	SDA
//PC5  	->	RES
//PE10  ->	DC
//PE12  ->	CS
//PE15  ->	BL
//在lcd_init.h中定义这些引脚
//在lcd.h查看可调用的函数
//cubemx的GPIO开启对应引脚，默认设置即可使用

void screen_init(void){
	LCD_Init();//LCD初始化
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
}

void screen_task(void *pvParameters){
	vTaskDelay(1000);


	while(1){
			
	}

	vTaskDelete(screen_task_handler);
}

void screen_task_create(void){
	xTaskCreate((TaskFunction_t ) 	screen_task,			//任务函数
				(const char *)		"screen_task",	  		//任务名字
				(uint32_t) 			screen_task_Size,    	//任务堆栈大小
				(void* )			NULL,					//传递给任务参数的指针参数
				(UBaseType_t) 		screen_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&screen_task_handler 	//任务句柄
				);		
}

