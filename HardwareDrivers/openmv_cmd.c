#include "openmv_cmd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

uint8_t openmv_rxbuff[MVBUFFERSIZE] = {0};
uint8_t mv_rxmsg = 0;
uint8_t mv_rxflag = 0;
uint8_t mv_rxArmflag = 0;
uint8_t mv_mode = 0;
uint8_t resend_cnt = 0;
#define CALLBACK_TIMEOUT	200		//ms
#define RESEND_TIME		3

void openmv_init(void)
{
	__HAL_UART_ENABLE_IT(&OPENMV_UART, UART_IT_IDLE);	//空闲中断
	HAL_UART_Receive_DMA(&OPENMV_UART,openmv_rxbuff,MVBUFFERSIZE);
}

uint8_t openmv_cmd(MV_CMD_t cmdsta)
{
	uint8_t command[5];
	uint16_t timeout = CALLBACK_TIMEOUT;
	resend_cnt = 0;
	command[0] = HeadByte1;
	command[1] = HeadByte2;
	command[3] = StatusByte;
	command[4] = EndByte;
	switch(cmdsta)
	{
		case bluemode:
		{
			command[2] = CMD_BLMOD;
			break;
		}
		case redmode:
		{
			command[2] = CMD_REMOD;
			break;
		}
		case disc:
		{
			command[2] = CMD_DISC;
			break;
		}
		case disc_off:
		{
			command[2] = DISC_OFF;
			break;
		}
		case step:
		{
			command[2] = CMD_STEP;
			break;
		}
		case colu:
		{
			command[2] = CMD_COLU;
			break;
		}
		case catc:
		{
			command[2] = CMD_CATH;
			break;
		}
		case stktsf:
		{
			command[2] =  CMD_STKTS;
			break;
		}
		case loose:
		{
			command[2] = CMD_LOSE;
			break;
		}
		case loose2:
		{
			command[2] = CMD_LOSE2;
			break;
		}
		case loose3:
		{
			command[2] = CMD_LOSE3;
			break;
		}
		case restart:
		{
			command[2] = CMD_RESART;
			break;
		}
		case loose4:
		{
			command[2] = CMD_LOSE4;
			break;
		}
		default:{}
	}
	mv_mode = command[2];
	//第一次正常往下运行，遇到goto语句才会跳转到此
	ReSend:
    HAL_UART_Transmit(&OPENMV_UART,command,5,1000);
	while(timeout > 0)		//
	{
		vTaskDelay(1);
		timeout--;
		if(openmv_getrxsta() == 1)
		{
			if(openmv_getrxmsg() == 1)	//Status = 1,cmd recvied
			{
				return 1;
			}
			else 
			{
				//never reach here
			}
		}
	}
	
	if(resend_cnt < RESEND_TIME)	//try again
	{
		resend_cnt++;
		timeout = CALLBACK_TIMEOUT;
		goto ReSend;
	}
	return 0;
}

// 获取芯片接收状态，1 接收成功 0 接受失败
uint8_t openmv_getrxsta(void)
{
	if(mv_rxflag)
	{
		mv_rxflag = 0;
		return 1;
	}
	else return 0;
}

// 获取mv应答内容
uint8_t openmv_getrxmsg(void)
{
	uint8_t temp = 0;
	if(mv_rxmsg)
	{
		temp = mv_rxmsg;
		mv_rxmsg = 0;
		return temp;
	}
	return 0;
}
uint8_t openmv_getArmsta(void)
{
	if(mv_rxArmflag)
	{
		mv_rxArmflag = 0;
		return 1;
	}
	else return 0;
}
uint16_t AngFerq[5] = {0};
uint16_t *openmv_getrxAng(void)
{
	return AngFerq;
}

uint8_t openmv_getresult(void)
{
	if(mv_rxflag)	//readonly not clear
	{
		return openmv_getrxmsg();
	}
	return 0;
}

void MV_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
	{
		if(RESET != __HAL_UART_GET_FLAG(&OPENMV_UART, UART_FLAG_IDLE))   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(&OPENMV_UART);                     //清楚空闲中断标志（否则会一直不断进入中断）
            MV_UART_IDLECallback(huart);                          //调用中断处理函数
        }
	}
}

// UART空闲表示一帧接收完成，解包，并置mv_rxflag为1
void MV_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	HAL_UART_DMAStop(&OPENMV_UART);
	mv_unpacked(openmv_rxbuff);
	mv_rxflag = 1;
	HAL_UART_Receive_DMA(&OPENMV_UART,openmv_rxbuff,MVBUFFERSIZE);
}

// 分类解析数据帧
void mv_unpacked(uint8_t * buf)
{
	if((buf[0] == HeadByte1) && 
	(buf[1] == HeadByte2) && 
	(buf[2] == 0x01) &&
	(buf[4] == EndByte))
	{
		mv_rxmsg = buf[3];
	}
	
	else if((buf[0] == HeadByte_Arm1) && 
	(buf[1] == HeadByte_Arm2) && 
	(buf[2] == 0x01) &&
	(buf[11] == EndByte))
	{
		mv_rxmsg = 128;
		AngFerq[0] = (uint16_t)(buf[4]<<8)|buf[3];
		AngFerq[1] = (uint16_t)(buf[6]<<8)|buf[5];
		AngFerq[2] = (uint16_t)(buf[8]<<8)|buf[7];
		AngFerq[3] = (uint16_t)(buf[10]<<8)|buf[9];
		mv_rxArmflag = 1;
	}

}

