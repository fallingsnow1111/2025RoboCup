#include "sensor.h"

GPIO_PinState read_iosensor(SENSOR_t sensor_id)
{
	switch(sensor_id)
	{
		case lir1:return READ_L1_IR;
		case rir1:return READ_R1_IR;
		case lir2:return READ_L2_IR;
		case rir2:return READ_R2_IR;
		case start:return READ_start_IR;
		
		case in1:return READ_TK_IN1;
		case in2:return READ_TK_IN2;
		case mid1:return READ_TK_MID1;
		case mid2:return READ_TK_MID2;
		case ex1:return READ_TK_EX1;
		case ex2:return READ_TK_EX2;

		case tk1:return READ_TK1;
		case tk2:return READ_TK2;
		case tk3:return READ_TK3;
		case tk4:return READ_TK4;
		case tk5:return READ_TK5;
		case tk6:return READ_TK6;
		
		case photogate: return READ_PHOTOGATE;
		case color_select: return READ_COLOR_SELECT;
	}
}


