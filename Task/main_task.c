#include "main_task.h"
#include "math.h"
#include "imu.h"

/*MAX_PRIORITIES为16*/
//main_task任务句柄
TaskHandle_t main_task_handler;

#define main_task_Size 2048
//main_task任务优先级
#define main_task_Priority 13
#define pi 3.1415926
//粗定位电机旋转角度定义

#define straight_run_mode  			1
#define straight_run_smooth_mode 	2
#define translate_run_mode 			3
#define rot_run_mode       			4

#define MODE_CHANGE_TIME 			300

#define Straight_Speed 				30
int CHECK_Speed = 					50;

#define Translate_Speed 			18
#define IS_TEN_PERCRENT 			1

#define  Actual_L 66 				//1000脉冲对应的长度

#define LOAD_BLOCK while(load2_check_flag() == 0)

float realdis2angle(float real_distace)//mm单位
{
	return (real_distace/(3.14*80)) *360;
}

void IsReady(void)
{
	while(1)
	{
		if(read_iosensor(start) == 0)
			break;
		vTaskDelay(50);
	}
}

#define HOLE_SPEED 5
#define STACK_SPEED 10
#define HOLE_ANGEL -1
#define HOLE_01 135
#define HOLE_12 90
#define HOLE2_TO_HOLE3 117
#define EDGE2_TO_MID 360
#define STACK_DIS 210


//直行速度超过65会卡死，转弯推荐速度30
//速度为 正 表示：直行、左转、逆时针旋转
struct Main_Task_Fast_Mapping_inf Main_Task[50] =                 
{//				模式		 	距离（单位mm）   旋转角度  		 速度	   	 	 加速度		加速度比率	  减速度	 减速度比率
/*1*/	{translate_run_mode,			630,		0,		-Translate_Speed												},
/*2*/	{straight_run_mode,				4200,		0,			35,													}, 
/*3*/	{translate_run_mode,			775,		0,			15,					50, 		0.3,		50,		0.60}, 
/*4*/	{rot_run_mode,					0,			90,		-Straight_Speed-10,											},			
/*5*/	{straight_run_smooth_mode,		1700,		0,			40,				50,		0.45,			80,    		0.52},
/*6*/	{straight_run_mode,				HOLE_01,	0,			-HOLE_SPEED,		30,		0.3,			15,    		0.52},
/*7*/	{translate_run_mode,			200,		0,			-15															},
/*8*/	{straight_run_mode,				STACK_DIS,	0,		STACK_SPEED,				60,		0.45,			60,    	0.52},
/*9*/	{straight_run_mode,				2600,		0,			-30,		100,	0.3,			80,    	0.52	},
/*10*/	{translate_run_mode,			475,		0,			15															},
/*11*/	{straight_run_smooth_mode,		400,		0,		-Straight_Speed,		80,		0.4,			80,    	0.52	},
/*12*/	{translate_run_mode,			500,		0,			15															},
/*13*/	{translate_run_mode,			350,		0,			15															},
/*14*/	{rot_run_mode,					0,			-0.5,		-Straight_Speed-10											},
/*15*/	{straight_run_mode,				HOLE_12,			0,	-HOLE_SPEED,		30,		0.3,			15,    	0.52	},
/*16*/	{straight_run_mode,			HOLE2_TO_HOLE3,		0,	-HOLE_SPEED,		30,		0.3,			15,    	0.52	},
/*17*/	{rot_run_mode,					0,			-90,		-Straight_Speed-10,											},
/*18*/	{straight_run_mode,			EDGE2_TO_MID,		0,	HOLE_SPEED,		30,		0.3,			15,    	0.52	},
/*19*/	{straight_run_mode,				26,				0,		HOLE_SPEED,		30,		0.3,			15,    		0.52},
/*20*/	{straight_run_mode,			STACK_DIS*2,	0,			-STACK_SPEED,				60,		0.45,			60,    	0.52},
/*21*/	{straight_run_smooth_mode,		300,		0,			-40,				50,		0.45,			80,    		0.52},
		{straight_run_smooth_mode,		4300,		0,		Straight_Speed+10,			80,			0.45,		100,    0.8},
};
struct Main_Task_Fast_Mapping_inf R_Main_Task[50] =                 
{//				模式		 	距离（单位mm）   旋转角度  		 速度	   	 	 加速度		加速度比率	  减速度	 减速度比率
/*1*/	{translate_run_mode,			630,		0,			-15												},
/*2*/	{straight_run_mode,				4200,		0,		-Straight_Speed,												}, 
/*3*/	{translate_run_mode,			775,		0,			30,					50, 		0.3,		50,		0.60}, 
/*4*/	{rot_run_mode,					0,			90,		-Straight_Speed-10,											},			
/*5*/	{straight_run_smooth_mode,		1500,		0,			-30,				50,		0.45,			80,    		0.52},
/*6*/	{straight_run_mode,				HOLE_01,	0,			-HOLE_SPEED,		30,		0.3,			15,    		0.52},
/*7*/	{translate_run_mode,			200,		0,			-15															},
/*8*/	{straight_run_mode,				STACK_DIS,		0,		STACK_SPEED,				60,		0.45,			60,    	0.52	},
/*9*/	{straight_run_smooth_mode,		2000,		0,			30,		100,	0.3,			80,    	0.52	},
/*10*/	{translate_run_mode,			740,		0,			15															},
/*11*/	{straight_run_smooth_mode,		400,		0,		-Straight_Speed,		80,		0.4,			80,    	0.52	},
/*12*/	{translate_run_mode,			500,		0,			15															},
/*13*/	{translate_run_mode,			350,		0,			15															},
/*14*/	{rot_run_mode,					0,			-0.5,		-Straight_Speed-10											},
/*15*/	{straight_run_mode,				HOLE_12,			0,	-HOLE_SPEED,		30,		0.3,			15,    	0.52	},
/*16*/	{straight_run_mode,			HOLE2_TO_HOLE3,		0,	-HOLE_SPEED,		30,		0.3,			15,    	0.52	},
/*17*/	{rot_run_mode,					0,			-90,		-Straight_Speed-10,											},
/*18*/	{straight_run_mode,			EDGE2_TO_MID,		0,		15,		30,		0.3,			15,    	0.52	},
/*19*/	{straight_run_mode,				26,			0,			HOLE_SPEED,		30,		0.3,			15,    		0.52},
/*20*/	{straight_run_smooth_mode,		300,		0,			30,				50,		0.45,			80,    		0.52},
		{straight_run_smooth_mode,		4300,		0,		-Straight_Speed-10,			80,			0.45,		100,    0.8},
};
#define PI 3.1415926

void main_task(void *pvParameters){
	while(Init_Flag == 0){//等待任务初始化结束
		vTaskDelay(20);
	}
	IsReady();//使用光电开关控制
	
	while(1)
	{
		/*************************调试区域*************************/

		load2_find_zero();
		ALL_Main_Task();
		
//		find_Disc();
//		Disc_catch();
//		Disc2Stairs();
//		Stair_catch();
//		Stairs2Circular();
//		Pillar_catch();
//		Blue_colu2stack();
//		Blue_stack();
//		BTrk09scor();
//		Blue_go_home();
//		while(1)
			vTaskDelay(5);	
		vTaskDelete(main_task_handler);
	}
}
void ALL_Main_Task(void)
{
		if(!READ_COLOR_SELECT)
		{
			Blue_Main_Task();
		}
		if(READ_COLOR_SELECT)
		{
			Red_Main_Task();
		}
}
void Red_Main_Task(void)
{
		openmv_cmd(redmode);
		R_find_Disc();
		Disc_catch();
		R_Disc2Stairs();
		Stair_catch();
		Stairs2Circular();
		Pillar_catch();
		Blue_colu2stack();
		Blue_stack();
		BTrk09scor();
		Red_go_home();
}

void Blue_Main_Task(void)
{
		openmv_cmd(bluemode);
		find_Disc();
		Disc_catch();
		Disc2Stairs();
		Stair_catch();
		Stairs2Circular();
		Pillar_catch();
		Blue_colu2stack();
		Blue_stack();
		BTrk09scor();
		Blue_go_home();
}
	
void Fast_Mapping_Run(struct Main_Task_Fast_Mapping_inf Task_inf){
	#if IS_TEN_PERCRENT == 1
	Task_inf.Task_speed = Task_inf.Task_speed * 50;
	
	#endif
	switch(Task_inf.run_mode){
		case straight_run_mode:
			//换算
			Task_inf.Task_angle=(Task_inf.Task_angle/(3.14*80)) *360;
			//清空脉冲数
			Set_Motors_Zero(&Motor_1);
			Motor_mecanum_wheel_pid_straight_run(Task_inf.Task_angle,Task_inf.Task_speed,0);
			break;
		case straight_run_smooth_mode:
			//换算
			Task_inf.Task_angle=(Task_inf.Task_angle/(3.14*80)) *360;
			//清空脉冲数
			Set_Motors_Zero(&Motor_1);
			Motor_mecanum_wheel_pid_straight_run_smooth(Task_inf.Task_angle,Task_inf.Task_speed,0,Task_inf.Speed_Acc,Task_inf.Accelerate,Task_inf.Speed_Dec,Task_inf.Decelerate);
			break;
		case translate_run_mode:
			//换算
			Task_inf.Task_angle=1.463*Task_inf.Task_angle+7.767;
			//清空脉冲数
			Set_Motors_Zero(&Motor_1);
			Motor_mecanum_wheel_pid_translate_run(Task_inf.Task_angle,Task_inf.Task_speed,0);
			break;
		case rot_run_mode:
			vTaskDelay(100);
			Set_Motors_Zero(&Motor_1);
			Motor_mecanum_wheel_pid_rot_run(Task_inf.Turn_angle,0);
			break;
	}
}

void Motor_stop(void)
{
	Motor_mecanum_wheel_speed_run(0,0,0,0);
}

//自旋转对齐边线，消除旋转误差
void find_bar_quad(void)
{
	front:
	Motor_mecanum_wheel_speed_run(0,-300,0,0);
	while(1)
	{
		if(read_iosensor(ex1) == 0 || read_iosensor(ex2) == 0)
		{
			Motor_stop();
			break;
		}
	}
	while(1)
	{
		//逆时针旋转为正
		if(read_iosensor(ex1) == 1 && read_iosensor(ex2) == 0)
		{
			Motor_mecanum_wheel_speed_run(0,0,150,0);
		}
		else if(read_iosensor(ex1) == 0 && read_iosensor(ex2) == 1)
		{
			Motor_mecanum_wheel_speed_run(0,0,-150,0);
		}
		else if(read_iosensor(ex1) == 1 && read_iosensor(ex2) == 1)
		{
			goto front;
		}
		else if(read_iosensor(ex1) == 0 && read_iosensor(ex2) == 0)
		{
			Motor_stop();
			vTaskDelay(50);
			if(read_iosensor(ex1) == 0 && read_iosensor(ex2) == 0)
			{
				Motor_stop();
				break;
			}
			else
			{
				Motor_mecanum_wheel_speed_run(0,-100,0,0);
			}
		}
	}
}
//行进间走歪了之后拉出来再对齐
void clab_bar_quad(void)
{
	if(read_iosensor(mid1) == 0 || read_iosensor(mid2) == 0)		//have one
	{
		Motor_mecanum_wheel_speed_run(0,100,0,0);
		while(1)
		{
			if(read_iosensor(mid1) == 1 && read_iosensor(mid2) == 1)
			{
				Motor_stop();
				break;
			}
		}
		find_bar_quad();
		
	}
	//这不是已经对齐了吗
	else if(read_iosensor(ex1) == 0 && read_iosensor(ex2) == 0 && read_iosensor(mid1) == 1 && read_iosensor(mid2) == 1)
	{
		Motor_mecanum_wheel_speed_run(0,100,0,0);
		vTaskDelay(300);
		Motor_stop();
		find_bar_quad();
	}
	gyro_set_0();
}

void bar_edge(uint8_t edge)	//edge:1~left	2~right
{
	if(edge == 1)
	{
		Motor_mecanum_wheel_speed_run(300,0,0,0);
		while(1)
		{
			if(read_iosensor(lir1) == 1 && read_iosensor(rir1) == 0)
			{
				Motor_stop();
				break;
			}
		}
	}
	else if(edge == 2)
	{
		Motor_mecanum_wheel_speed_run(-300,0,0,0);
		
		while(1)
		{
			if(read_iosensor(lir1) == 0 && read_iosensor(rir1) == 1)
			{
				Motor_stop();
				break;
			}
		}
	}

}

void find_colu(void)
{
	while(1)
	{
		if(read_iosensor(lir2) == 1 && read_iosensor(rir2) == 0)
		{
			Motor_mecanum_wheel_speed_run(-200,0,0,0);
		}
		else if(read_iosensor(lir2) == 0 && read_iosensor(rir2) == 1)
		{
			Motor_mecanum_wheel_speed_run(200,0,0,0);
		}
		else if(read_iosensor(lir2) == 1 && read_iosensor(rir2) == 1)
		{
			Motor_mecanum_wheel_speed_run(0,-200,0,0);
		}
		else if(read_iosensor(lir2) == 0 && read_iosensor(rir2) == 0)
		{
			Motor_stop();
			break;
		}
	}
}

//出门并找到圆盘机
void R_find_Disc(void)
{
	arm_action_group_run(0);
	
//	Fast_Mapping_Run(Main_Task[16]);
	Fast_Mapping_Run(R_Main_Task[0]);
//	Fast_Mapping_Run(Main_Task[3]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[20]);
	
	find_bar_quad();
}
void find_Disc(void)
{
	arm_action_group_run(0);
	
//	Fast_Mapping_Run(Main_Task[16]);
	Fast_Mapping_Run(Main_Task[0]);
//	Fast_Mapping_Run(Main_Task[3]);
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[21]);
	
	find_bar_quad();
}

#define DISC_BALLNUM	6

void Disc_catch(void)
{
	uint8_t openmvrec = 0;
	uint8_t err_cnt = 0;
	
	vTaskDelay(500);
	for(uint8_t i = 1;i < DISC_BALLNUM+2;i++)
	{
		arm_action_group_run(10);
		vTaskDelay(500);
		openmv_cmd(disc);

		while(1)		//
		{
			if(openmv_getrxsta() == 1)
			{
				openmvrec = openmv_getrxmsg();
				if(openmvrec == 0x02) break;
				else if(openmvrec == 0x03)
				{
					//printf("timeOUT\n");
					goto Timeout;//?
				}
			}
		}

		vTaskDelay(10);
		arm_action_group_run(11);
		vTaskDelay(50);
		openmv_cmd(loose);
		vTaskDelay(300);
		
		if(i == 2)
		{
			load2_enable_flag(1);
		}
		else
		{
			while(load2_check_flag() == 0);
			load2_enable_flag(1);
		}
		vTaskDelay(50);
		
	}
	arm_action_group_run(0);
	vTaskDelay(200);
	return;
	
	Timeout:
	
	arm_action_group_run(11);
	arm_action_group_run(0);
	
	return;
}

//离开圆盘机开到阶梯平台
void R_Disc2Stairs(void)
{
	Fast_Mapping_Run(R_Main_Task[2]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[19]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[2]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[3]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[3]);
	vTaskDelay(100);
	Fast_Mapping_Run(R_Main_Task[4]);
	
	find_bar_quad();
	clab_bar_quad();
	vTaskDelay(100);
	bar_edge(1);
	vTaskDelay(300);
	Fast_Mapping_Run(R_Main_Task[18]);
}
void Disc2Stairs(void)
{
	Fast_Mapping_Run(Main_Task[2]);
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[20]);
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[2]);
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[3]);
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[3]);
	vTaskDelay(100);
	
	Fast_Mapping_Run(Main_Task[4]);
	
	find_bar_quad();
	clab_bar_quad();
	vTaskDelay(100);
	bar_edge(1);
	vTaskDelay(300);
	Fast_Mapping_Run(Main_Task[18]);
}

#define STEP_BALLNUM	2
#define STEP_HOLENUM	8
void Stair_catch(void)
{
	uint8_t noball_flag = 0,ball_cnt = 0;
	uint8_t mv_msg = 0; 
	
	openmv_cmd(loose2);
	arm_action_group_run(30);
	
	// i就是当前所在孔位
	for(uint8_t i = 1;i < STEP_HOLENUM + 1; i++)
	{
		if(i != 1)vTaskDelay(180);

		vTaskDelay(50);
		switch(i)
		{
			case 7:
			{
				arm_action_group_run(34);
				break;
			}
		}
		openmv_cmd(step);

		while(1)
		{
			if(openmv_getrxsta() == 1)
			{
				mv_msg = openmv_getrxmsg();
				if(mv_msg == 0x02)break;
				else if(mv_msg == 0x03)
				{
					noball_flag = 1;
					break;
				}
			}
		}
		
		if(noball_flag == 0)	//is ball
		{
			openmv_cmd(catc);
			vTaskDelay(200);
			if(i == 1 || i == 2)	arm_action_group_run(32);
			else if(i > 2 && i < 7)	arm_action_group_run(33);
			else if(i == 7|| i == 8)arm_action_group_run(35);
			
			vTaskDelay(50);
			openmv_cmd(loose2);
			vTaskDelay(400);
			clab_bar_quad();
			ball_cnt++;
			while(load2_check_flag() == 0);
			load2_enable_flag(1);
			
			if(ball_cnt == STEP_BALLNUM)	//all ball get,out
			{
				arm_action_group_run(0);
				clab_bar_quad();

				bar_edge(2);
				clab_bar_quad();
				Fast_Mapping_Run(Main_Task[17]);
				break;
			}
			if(ball_cnt == 1)
			{
				if(i == 1 || i == 2)		ball_step_putback(1);
				else if(i > 2 && i < 7)		ball_step_putback(2);
				else if(i == 7|| i == 8)	ball_step_putback(3);
			}
			
			//把球放进仓库后机械臂的复位
			switch(i)			
			{
				case 1:
				{
					arm_action_group_run(30);
					break;
				}
				case 2:
				{
					arm_action_group_run(31);
					break;
				}
				case 3:
				{
					arm_action_group_run(31);				
					break;				
				}
				case 4:
				{
					clab_bar_quad();
					arm_action_group_run(31);
					break;
				}
				case 5:
				{	
					arm_action_group_run(31);					
					break;
				}
				case 6:
				{
					//变高度
					arm_action_group_run(31);	
					break;
				}
				case 7:
				{
					arm_action_group_run(34);
					break;
				}
			}
			vTaskDelay(100);
		}
		else		//没球就调整机械臂准备运动到下一个孔位
		{
//			printf("not ball");
			noball_flag = 0;
			if(i == 2)
			{	
				arm_action_group_run(31);	//High1 to High2,lift
				clab_bar_quad();
			}
			else if(i == 4)
			{
				clab_bar_quad();
			}
		}
		switch(i)
		{
			case 1:
			{
				Fast_Mapping_Run(Main_Task[14]);
				break;
			}
			case 2:
			{
				Fast_Mapping_Run(Main_Task[15]);
				break;
			}
			case 3:
			case 4:
			case 5:
			{
				Fast_Mapping_Run(Main_Task[14]);
				break;
			}
			case 6:
			{
				clab_bar_quad();
				Fast_Mapping_Run(Main_Task[15]);
				break;
			}
			case 7:
			{
				Fast_Mapping_Run(Main_Task[14]);
				break;
			}
			case 8:
			{
				arm_action_group_run(0);
				clab_bar_quad();

				bar_edge(2);
				clab_bar_quad();
				Fast_Mapping_Run(Main_Task[17]);
				break;
			}
		}
	}		
}

// 把无效球放回阶梯平台
void ball_step_putback(uint8_t sta)
{
	//对应三个高度的放回
	if(sta == 1)		//P1 no ball
	{
		LOAD_BLOCK;
		load2_scores_outid(0x14);
		load2_enable_flag(2);
		LOAD_BLOCK;
		openmv_cmd(catc);
		vTaskDelay(100);
		arm_action_group_run(30);
		vTaskDelay(100);
		openmv_cmd(loose2);
		vTaskDelay(200);
	}
	else if(sta == 2)	//P1 is ball
	{
		LOAD_BLOCK;
		load2_scores_outid(0x14);
		load2_enable_flag(2);
		LOAD_BLOCK;
		openmv_cmd(catc);
		vTaskDelay(100);
		arm_action_group_run(31);
		vTaskDelay(100);
		openmv_cmd(loose2);
		vTaskDelay(200);
	}
	else if(sta == 3)
	{
		LOAD_BLOCK;
		load2_scores_outid(0x14);
		load2_enable_flag(2);
		LOAD_BLOCK;
		openmv_cmd(catc);
		vTaskDelay(100);
		arm_action_group_run(34);
		vTaskDelay(100);
		openmv_cmd(loose2);
		vTaskDelay(200);
	}
}

//先靠近绕桩，再矫正前后距离
void Stairs2Circular(void)
{
	Fast_Mapping_Run(Main_Task[11]);
	Fast_Mapping_Run(Main_Task[3]);
	Fast_Mapping_Run(Main_Task[3]);
	
	find_colu();	

	Motor_mecanum_wheel_speed_run(0,-100,0,0);
	while(1)
	{	
		if(read_iosensor(in1) == 0 || read_iosensor(in2) == 0)
		{
			Motor_stop();
			break;
		}
	}	
}

uint8_t Stop_flag = 0;

void Pillar_catch(void)
{
	float last_yaw1 = 0;
	uint8_t openmvrec = 0;
	
	openmv_cmd(loose4);
	arm_action_group_run(40);
	
	for(uint8_t i = 0;i < 2;i++)
	{
		openmv_cmd(colu);
		while(1)
		{
			last_yaw1 = imu.yaw_set;
			if(Stop_flag != 3)
			{
				if(openmv_getrxsta() == 1)
				{
					openmvrec = openmv_getrxmsg();
					if(openmvrec == 0x02)
					{
						Motor_stop();
						break;
					}
					else if(openmvrec == 0x03)
					{
						goto ColuTimeout;//?
					}
				}
				
				if(read_iosensor(in1) == 1 && read_iosensor(in2) == 0)
				{
					Motor_mecanum_wheel_speed_run(400,0,-600,0);
				}
				else if(read_iosensor(in1) == 0 && read_iosensor(in2) == 1)
				{
					Motor_mecanum_wheel_speed_run(400,0,-670,0);
				}
				else if(read_iosensor(in1) == 0 && read_iosensor(in2) == 0)
				{
					Motor_mecanum_wheel_speed_run(400,0,-645,0);			
				}
				else if(read_iosensor(in1) == 1 && read_iosensor(in2) == 1)
				{
					Motor_mecanum_wheel_speed_run(400,-45,-645,0);
				}
				switch(Stop_flag)
				{
					case 0:
					{
						if(imu.yaw_set <= -170)
							Stop_flag = 1;
						break;					
					}
					case 1:
					{
						if(imu.yaw_set >= 170)
							Stop_flag = 2;
						break;					
					}
					case 2:
					{
						if(last_yaw1 < 0 && imu.yaw_set >0)
							Stop_flag = 3;
						break;					
					}
					default:
						break;
				}
				if(Stop_flag == 3)
				{
					Motor_stop();
					gyro_set_0();
					break;
				}		
			}
			else
			{
				if(openmv_getrxsta() == 1)
				{
					openmvrec = openmv_getrxmsg();
					if(openmvrec == 0x03)
					{
						goto ColuTimeout;
					}
				}
			}
		}
	
		vTaskDelay(300);
		openmv_cmd(catc);
		vTaskDelay(200);

		arm_action_group_run(35);
		vTaskDelay(200);
		openmv_cmd(loose);
		while(load2_check_flag() == 0);
		load2_enable_flag(1);
		vTaskDelay(200);

		//printf("now_angle:%f\n",imu_angle);
		if(i == 0)
		{
			openmv_cmd(loose4);
			arm_action_group_run(40);
		}
	}

	arm_action_group_run(0);
	Circular();
	return;
	
	ColuTimeout:
	openmv_cmd(loose);
	arm_action_group_run(35);
	arm_action_group_run(0);
	Circular();
	return;
}

//绕桩+
void Circular(void)
{	
	float last_yaw = imu.yaw_set;
	while(1)
	{
		if(Stop_flag != 3)
		{	
			if(read_iosensor(in1) == 1 && read_iosensor(in2) == 0)
			{
				Motor_mecanum_wheel_speed_run(-400,0,600,0);
			}
			else if(read_iosensor(in1) == 0 && read_iosensor(in2) == 1)
			{
				Motor_mecanum_wheel_speed_run(-400,0,670,0);
			}
			else if(read_iosensor(in1) == 0 && read_iosensor(in2) == 0)
			{
				Motor_mecanum_wheel_speed_run(-400,20,645,0);			
			}
			else if(read_iosensor(in1) == 1 && read_iosensor(in2) == 1)
			{
				Motor_mecanum_wheel_speed_run(-400,-45,600,0);
			}
			
			switch(Stop_flag)
			{
				case 0:
					if(imu.yaw_set <= -170)
						Stop_flag = 1;
					break;
				case 1:
					if(imu.yaw_set >= 170)
						Stop_flag = 2;
					break;
				case 2:
					if(last_yaw > 0 && imu.yaw_set < 0 )
						Stop_flag = 3;
					else if(imu.yaw_set)
					break;				
				default:
					break;
			}
			if(Stop_flag == 3)
			{
				Motor_stop();
				gyro_set_0();
				break;
			}			
		}
	}
}


//立仓
void Blue_colu2stack(void)
{
	Fast_Mapping_Run(Main_Task[12]);
	Fast_Mapping_Run(Main_Task[3]);
	Fast_Mapping_Run(Main_Task[3]);
	
	find_bar_quad();
	clab_bar_quad();
	
	bar_edge(2);
	vTaskDelay(500);
}

uint8_t transfer_flag = 0;
uint16_t QRcode_store[3] = {0};

void Blue_stack(void)
{
	uint8_t mvrec = 0,hasball = 0;
	uint8_t j = 0;
	Fast_Mapping_Run(Main_Task[7]);
	
	for(uint8_t i = 0;i < 3;i++)
	{
		switch(i)
		{
			case 0:
				arm_action_group_run(51);
				openmv_cmd(loose3);
				arm_action_group_run(50);
				break;
			case 1:
				arm_action_group_run(54);
				openmv_cmd(loose3);
				arm_action_group_run(53);
				break;
			case 2:
				arm_action_group_run(57);
				openmv_cmd(loose3);
				arm_action_group_run(56);
				break;
		}
		
		vTaskDelay(200);
		hasball = 0;
		for(uint8_t retry = 0;retry < 3;retry ++)
		{
			vTaskDelay(300);
			openmv_cmd(stktsf);
			while(1)
			{
				if(openmv_getrxsta() == 1)
				{
					mvrec = openmv_getrxmsg();
					if(mvrec == 0x02)
					{
						hasball = 1;
						break;
					}
					else if(mvrec == 0x03)
					{
						hasball = 0;
						break;
					}
				}
			}
			if(hasball)
			{
				j = retry;
				break;
			}				
			else
			{
				//若走到最左边还没球就对齐然后退出
				if(retry == 2)
				{
					arm_action_group_run(0);
					Motor_mecanum_wheel_speed_run(0,200,0,0);
					vTaskDelay(500);
					Motor_stop();
					Fast_Mapping_Run(Main_Task[19]);
					find_bar_quad();
					break;
				}
				else
				{
					Fast_Mapping_Run(Main_Task[7]);
					clab_bar_quad();
				}
			}
		}
		
		if(hasball)
		{
			switch(i)
			{
				case 0:{
					find_bar_quad();
					arm_action_group_run(51);
					openmv_cmd(loose);
					vTaskDelay(100);
					arm_action_group_run(52);
					openmv_cmd(catc);
					vTaskDelay(100);
					arm_action_group_run(51);
					vTaskDelay(100);
					
					if(j == 2)
					{
						Motor_mecanum_wheel_speed_run(0,200,0,0);
						vTaskDelay(500);
						Motor_stop();
						Fast_Mapping_Run(Main_Task[19]);
						find_bar_quad();
					}
					
					bar_edge(2);
					clab_bar_quad();
					arm_action_group_run(52);
					openmv_cmd(loose);
					vTaskDelay(100);
					arm_action_group_run(51);
					Fast_Mapping_Run(Main_Task[7]);
					break;}
				case 1:{
					find_bar_quad();
					arm_action_group_run(54);
					openmv_cmd(loose);
					vTaskDelay(200);
					arm_action_group_run(55);
					vTaskDelay(100);
					openmv_cmd(catc);
					vTaskDelay(100);
					arm_action_group_run(54);
					
					if(j == 2)
					{
						Motor_mecanum_wheel_speed_run(0,200,0,0);
						vTaskDelay(500);
						Motor_stop();
						Fast_Mapping_Run(Main_Task[19]);
						find_bar_quad();
					}
					
					bar_edge(2);
					clab_bar_quad();
					arm_action_group_run(55);
					openmv_cmd(loose);
					vTaskDelay(100);
					arm_action_group_run(54);
					Fast_Mapping_Run(Main_Task[7]);
					break;}
				case 2:{
					find_bar_quad();
					arm_action_group_run(57);
					openmv_cmd(loose);
					vTaskDelay(200);
					arm_action_group_run(58);
					vTaskDelay(100);
					openmv_cmd(catc);
					vTaskDelay(250);
					arm_action_group_run(57);
					
					if(j == 2)
					{
						Motor_mecanum_wheel_speed_run(0,200,0,0);
						vTaskDelay(500);
						Motor_stop();
						Fast_Mapping_Run(Main_Task[19]);
						find_bar_quad();
					}
					
					bar_edge(2);
					clab_bar_quad();
					arm_action_group_run(58);
					openmv_cmd(loose);
					vTaskDelay(100);
					arm_action_group_run(57);
					break;}
			}
		}
		else				//not ball
		{
			bar_edge(2);
			clab_bar_quad();
			Fast_Mapping_Run(Main_Task[7]);
			vTaskDelay(100);
		}
	}
	openmv_cmd(loose);
	arm_action_group_run(0);
}
/*			c1	c2	c2
	floor3	31 	32	33
	floor2	21	22	23
	floor1	11	12	13
*/
void BTrk09scor(void)
{
	uint8_t qr_buff = 0;
	uint8_t qr_enflag = 0;	//取消二维码定义方式
	bar_edge(2);
	find_bar_quad();
	vTaskDelay(100);
	Fast_Mapping_Run(Main_Task[7]);

	arm_action_group_run(41);
	load2_scores_outid(0x33);
	
	for(uint8_t i = 0;i<3;i++)
	{
		find_bar_quad();
		clab_bar_quad();
		for(uint8_t j =0;j<3;j++)
		{
			openmv_cmd(loose);
			if(j != 2)		//qrcode
			{
				load2_scores_outid(0x33-i-(0x10*j));
			}
			else
			{
				if(qr_enflag)
				{
					qr_buff = qrcode_getRxbuff();
				}
				else
				{
					qr_buff = 0;
				}
				
				switch(qr_buff)
				{
					case '1':
					{
						load2_scores_outid(0x11);
						break;
					}
					case '2':
					{
						load2_scores_outid(0x12);
						break;
					}
					case '3':
					{
						load2_scores_outid(0x13);
						break;
					}
					default:
					{
						qr_enflag = 0;
						load2_scores_outid(0x33-i-0x10*j);
					}
				}
			}

			arm_action_group_run(110);
			LOAD_BLOCK;
			vTaskDelay(50);
			openmv_cmd(catc);
			vTaskDelay(100);
			arm_action_group_run(41);
			arm_action_group_run(70+j);
			arm_action_group_run(73+j);
			vTaskDelay(100);
			openmv_cmd(loose);
			vTaskDelay(100);
			
			arm_action_group_run(70+j);
			arm_action_group_run(41);
		}
		if(i == 2)break;
		Fast_Mapping_Run(Main_Task[7]);
		clab_bar_quad();
		vTaskDelay(100);
	}
	
	arm_action_group_run(0);
	if(read_iosensor(lir1) == 1 && read_iosensor(rir1) == 0)
	{
		Motor_mecanum_wheel_speed_run(-100,0,0,0);
		while(1)
		{
			if(read_iosensor(lir1) == 0 && read_iosensor(rir1) == 0)break;
		}
		Motor_stop();
	}
	
	bar_edge(1);
	clab_bar_quad();
	vTaskDelay(200);
}

void Red_go_home(void)
{
	arm_action_group_run(1);
	Fast_Mapping_Run(R_Main_Task[8]);
	Fast_Mapping_Run(R_Main_Task[9]);
}

void Blue_go_home(void)
{
	arm_action_group_run(1);
	Fast_Mapping_Run(Main_Task[12]);
	Fast_Mapping_Run(Main_Task[8]);
	Fast_Mapping_Run(Main_Task[9]);
}

void main_task_create(void){
	xTaskCreate((TaskFunction_t ) 	main_task,			//任务函数
				(const char *)		"main_task",	  	//任务名字
				(uint32_t) 			main_task_Size,    	//任务栈大小
				(void* )			NULL,				//传递给任务参数的指针参数
				(UBaseType_t) 		main_task_Priority,	//任务的优先级
				(TaskHandle_t *)	&main_task_handler 	//任务句柄
				);	
}
