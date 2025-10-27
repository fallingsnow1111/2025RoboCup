#ifndef __OPENMV_CMD_H__
#define __OPENMV_CMD_H__

#include "usart.h"

#define OPENMV_UART huart6

// 宏定义放在头文件里才能被其他文件引用
#define MVBUFFERSIZE 15

#define HeadByte1  0x2C      //传输数据帧帧头1
#define HeadByte2  0x12      //传输数据帧帧头2
#define CmdByte    0x01      //传输数据帧命令帧
#define StatusByte 0x00      //传输数据帧状态帧
#define EndByte    0x5B      //传输数据帧帧尾

#define HeadByte_Arm1	0x3C
#define HeadByte_Arm2	0x13

#define CMD_BLMOD	0x01
#define CMD_REMOD	0x02

#define CMD_DISC	0x11
#define DISC_OFF	0x12
#define CMD_STEP	0x21
#define CMD_COLU	0x31
#define CMD_STKTS	0x41

#define CMD_CATH	0xF0
#define CMD_LOSE	0xF1
#define CMD_LOSE2	0xF2
#define CMD_LOSE3	0xF3
#define CMD_LOSE4	0xF4

#define CMD_RESART	0xFF

typedef enum mv_cmd{bluemode = 1,redmode,restart,disc,disc_off,step,colu,stktsf,catc,loose,loose2,loose3,loose4}MV_CMD_t;

void openmv_init(void);
void MV_UART_IDLECallback(UART_HandleTypeDef *huart);
void MV_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
uint8_t openmv_getrxsta(void);
uint8_t openmv_getrxmsg(void);
void mv_unpacked(uint8_t * buf);
uint8_t openmv_cmd(MV_CMD_t cmdsta);
uint8_t openmv_getresult(void);
uint8_t openmv_getArmsta(void);
uint16_t *openmv_getrxAng(void);

#endif
