#include "start_task.h"

/************************************************
*注释main.c中的MX_FREERTOS_Init();
MAX_PRIORITIES为16
只运行一次的任务切记要删除
*************************************************/

/************************************************
使用USMART:
1、注释原先的USART2_IRQHandler();
2、注释MX_USART2_UART_Init()
3、回调与中断服务都在uart.c里面
*************************************************/


//start_task任务句柄
TaskHandle_t start_task_handler;
//start_task任务堆栈大小
#define start_task_Size 256
//start_task任务优先级
#define start_task_Priority 14

//load_task任务句柄
TaskHandle_t load_task_handler;
//start_task任务堆栈大小
#define load_task_Size 256
//start_task任务优先级
#define load_task_Priority 14

//user_init任务句柄
TaskHandle_t user_init_handler;
//user_init任务堆栈大小
#define user_init_Size 256
//user_init任务优先级
#define user_init_Priority 14

uint8_t Init_Flag = 0;

//打印所有的task信息	
void print_task(void)
{
	printf("\n\r");
	char pcWriteBuffer[512];
	vTaskList(pcWriteBuffer); // ??????
	printf("Task List:\r\n%s\r\n", pcWriteBuffer);
}	

void start_task(void *pvParameters){
	
	taskENTER_CRITICAL();          			//进入临界区

		user_init_create();				//用户外设初始化
		load_task_create();
		//led_task_create();				//led闪烁判断是否卡死
		main_task_create();				//主要任务开启
	
	
	
	taskEXIT_CRITICAL();            		//退出临界区

	vTaskDelete(NULL); 		//删除start_task任务
}

void load_task(void *pvParameters)
{
	while(1)
	{
		load2_main();
		vTaskDelay(10);
	}
}

void start_task_create(void){
	xTaskCreate((TaskFunction_t ) 	start_task,				//任务函数
				(const char *)		"start_task",	  		//任务名字
				(uint32_t) 			start_task_Size,    	//任务堆栈大小
				(void* )			NULL,					//传递给任务参数的指针参数
				(UBaseType_t) 		start_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&start_task_handler 	//任务句柄
				);	
}

void load_task_create(void){
	xTaskCreate((TaskFunction_t ) 	load_task,				//任务函数
				(const char *)		"load_task",	  		//任务名字
				(uint32_t) 			load_task_Size,    	//任务堆栈大小
				(void* )			NULL,					//传递给任务参数的指针参数
				(UBaseType_t) 		load_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&load_task_handler 	//任务句柄
				);
}				

void user_init(void *pvParameters){
		delay_init();			//delay_ms和delay_us初始化
		gyro_init(); 			//陀螺仪串口初始化
		qrcode_init();			//二维码模块初始化
		PID_Init();				//PID参数初始化
		motor_usart_init();		//电机串口初始化
		rfid_init();
		arm_init();
		openmv_init();
		load2_init();
		buzzer_complete_flag = 1;
		Init_Flag = 1;
	
	vTaskDelete(NULL); 		//初始化结束删除user_init任务
}

void user_init_create(void){
	xTaskCreate((TaskFunction_t ) 	user_init,				//任务函数
				(const char *)		"user_init",	  		//任务名字
				(uint32_t) 			user_init_Size,    		//任务堆栈大小
				(void* )			NULL,					//传递给任务参数的指针参数
				(UBaseType_t) 		user_init_Priority,		//任务的优先级
				(TaskHandle_t *)	&user_init_handler 		//任务句柄
				);		
}



