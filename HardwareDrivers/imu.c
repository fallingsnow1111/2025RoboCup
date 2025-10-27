#include "imu.h"
/**************************/
/*		TX —— PD5		  */
/*		RX ——	PD6		  */
/**************************/


#define IS_HWT101 1//使用的是否是HWT101,否则默认为IMU601
const uint8_t Cmd_Set_Zero_HWT101[5] = {0xFF, 0xAA, 0x76, 0x00, 0x00};

#define Gyro huart2 
uint8_t imu_rx_buf[IMU_BUFFER_SIZE] = {0};//陀螺仪初始信息接收buf
uint8_t imu_rx_len = 0;//陀螺仪初始信息接收buf长度
volatile struct Imu imu={0};//陀螺仪信息

void gyro_init(void){
	USART2->ICR = USART2->ICR | (0x01<<4);
	__HAL_UART_ENABLE_IT(&Gyro, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&Gyro,imu_rx_buf,IMU_BUFFER_SIZE);
	//陀螺仪置零
	do{
		HAL_UART_Transmit(&Gyro,(uint8_t*)Cmd_Set_Zero_HWT101,sizeof(Cmd_Set_Zero_HWT101),0xffff);
	}
	while(imu.yaw_set != 0);
}

/****************************陀螺仪软校准****************************/
//输出当前角度与目标角度的最小夹角
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;				//实际所需转的角度
	if(need2Turn>180)	need2Turn -= 360;
  	else if(need2Turn<-180)	need2Turn += 360;
	
  	return need2Turn;		
}

void mpuZreset(float sensorangle ,float referangle)
{	
	imu.compensateZ=need2turn(sensorangle,referangle);
}

void gyro_set_0(void){
	float mpuZ_reset_val = 0;
	vTaskDelay(50);

	for (int i=0; i<10; i++)
	{
		vTaskDelay(20);
		mpuZ_reset_val += imu.yaw;
	}
	
	mpuZ_reset_val /= 10;
	mpuZreset(mpuZ_reset_val, 0);
}

void gyro_set_reduce_90(void){
	imu.compensateZ -= 90;
	if(imu.compensateZ <= -360)imu.compensateZ += 360;
}
void gyro_set_plus_90(void){
	imu.compensateZ += 90;
	if(imu.compensateZ >= 360)imu.compensateZ -= 360;
}
/****************************陀螺仪软校准****************************/

void USART2_IRQHandler(void)
{
	/*定义变量*/
	uint32_t flag_idle = 0;
	uint32_t temp = 0;
	
	/*接收部分*/
	flag_idle = __HAL_UART_GET_FLAG(&Gyro,UART_FLAG_IDLE);//获取IDLE标志位
	if((flag_idle != RESET))//IDLE标志被置位
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&Gyro);//清除IDLE标志位
		HAL_UART_DMAStop(&Gyro); //停止DMA接收
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//获取DMA中未传输的数据个数
		imu_rx_len = IMU_BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
		
		#if IS_HWT101 == 1
		
		/*先定位数据帧*/
		int ptr = 0;
		int max_ptr = imu_rx_len - 1;
		while(ptr < max_ptr)
		{
			if((imu_rx_buf[ptr] == 0x55) && (imu_rx_buf[ptr+1] == 0x53))
			{
				if((imu_rx_len - ptr) >= 11)
				{
					uint8_t sum = 0;
					for (int i=0; i<10; i++)
						sum += imu_rx_buf[i+ptr]; 
					if (sum == imu_rx_buf[10+ptr])
					{
						//根据陀螺仪确定yaw
						imu.yaw   = 180.0* (float)((imu_rx_buf[7+ptr]<<8)|imu_rx_buf[6+ptr])/32768.0f;
						if(imu.yaw>=360) imu.yaw -= 360;
						else if(imu.yaw<=0) imu.yaw += 360;
						
						imu.yaw_set = imu.yaw + imu.compensateZ;//设定上电时初始角为0
						//yaw_set [-180,180]逆时针方向变化
						if(imu.yaw_set>=180) imu.yaw_set -= 360;
						else if(imu.yaw_set<=-180) imu.yaw_set += 360;
						break;
					}
				}
			}
			ptr++;
		}

		#else
		
		/*判断是否符合协议且为非空信息*/
		if(imu_rx_buf[0] == 0x55 && imu_rx_len != 0)
		{
			uint8_t sum = 0;
			for (int i=0; i<10; i++)
				sum += imu_rx_buf[i];
			if (sum == imu_rx_buf[10])
			{
				if (imu_rx_buf[2] == 0X01)
				{
					//根据陀螺仪的安装方向确定roll和pitch
					imu.roll  = 180.0* (float)((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0f;  
					imu.pitch = -180.0*(float)((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0f;
					imu.yaw   = 180.0* (float)((imu_rx_buf[9]<<8)|imu_rx_buf[8])/32768.0f;
					if(imu.yaw>=360) imu.yaw -= 360;
					else if(imu.yaw<=0) imu.yaw += 360;
					
					imu.yaw_set = imu.yaw + imu.compensateZ;//设定上电时初始角为0
					//yaw_set [-180,180]逆时针方向变化
					if(imu.yaw_set>=180) imu.yaw_set -= 360;
					else if(imu.yaw_set<=-180) imu.yaw_set += 360;
				}
			}
		}
		#endif
		
		//清空imu_rx_buf和imu_rx_len
		memset(imu_rx_buf,0,IMU_BUFFER_SIZE);
		imu_rx_len = 0;
	}
	HAL_UART_Receive_DMA(&Gyro,imu_rx_buf,IMU_BUFFER_SIZE);	
	HAL_UART_IRQHandler(&Gyro); 
}