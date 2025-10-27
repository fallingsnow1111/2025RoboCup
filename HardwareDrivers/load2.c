#include "load2.h"
#include "stepmotor_cmd.h"
#include "sensor.h"
#include "stdio.h"
#include "rfid.h"
#include "FreeRTOS.h"
#include "task.h"

#include "string.h"
#define GAP_PUSE	1280		//16细分时
#define SCORE_NUM2	12
#define STPMOT_BLOCK	while(stpmot_work_sta() == 1)

void load2_init(void)
{
	stpmot_init();
}

uint16_t load2_timeout = 0;
volatile uint8_t load2_sta = 0;	//仓库状态机
volatile uint8_t ball_outid2 = 0;
volatile uint8_t rfid_dat = 0;

void load2_main(void)
{
	if(load2_sta == 1)
	{
		load2_timeout = 300;
		while(1)
		{
			if(rfid_getsta() == 1)
			{
				load2_scores_in(rfid_getid());
				break;
			}
			if(load2_timeout > 0){load2_timeout--;}
			else if(load2_timeout == 0)
			{
				rfid_getsta();
				load2_scores_in(0xff);
				break;
			}
			vTaskDelay(5);
		}
		load2_sta = 0;
	}
	else if(load2_sta == 2)
	{
		load2_scores_out(ball_outid2);
		load2_sta = 0;
	}
	else
	{
		load2_sta = 0;
	}
		
//		load_last_flag();
}

// 检查是否在执行出入仓 0 是 1 否
uint8_t load2_check_flag(void)
{
	if(load2_sta)return 0;
	else return 1;
}

// 标志入仓
void load2_enable_flag(uint8_t sta)
{
	load2_sta = sta;
}

uint8_t iosta = 0;

void load2_find_zero(void)
{
	stpmot_find_zero();
	while(1)
	{
		if(read_iosensor(photogate) == 1)
		{
			stpmot_stop_find_zero();
			break;
		}
	};
}

// 顺时针转一个孔位
void load2_next_flag(void)
{
	stpmot_to_degree(GAP_PUSE,1);
	STPMOT_BLOCK;
}

void load2_last_flag(void)
{
	stpmot_to_degree(GAP_PUSE,0);
	STPMOT_BLOCK;
}

// 顺时针转n个孔位
void load2_next_nflag(uint8_t cnt)
{
	stpmot_to_degree(GAP_PUSE*cnt,1);
	//printf("stpmot_send!\n");
	STPMOT_BLOCK;
}

void load2_last_nflag(uint8_t cnt)
{
	stpmot_to_degree(GAP_PUSE*cnt,0);
	//printf("stpmot_send!\n");
	STPMOT_BLOCK;
}

volatile uint8_t scoreid2[SCORE_NUM2] = {0};
uint8_t scorecnt2 = 0;

int8_t place_now2 = 0;			//电机当前指向孔位
uint8_t target_place2 = 0;		
uint8_t printfbuff[128] = {0};

// 算最短路径转到目标位置
void load2_to_place(uint8_t place)		//place-->place_now2
{
//	sprintf(printfbuff,"placenow:%d,place:%d\n",place_now2,place);
//	Serial_Send(&huart1,printfbuff);
//	memset(printfbuff,0,sizeof(printfbuff));
	int8_t place_det = 0;
	
	if(place > 11 || place <  0)
	{
		//printf("toplace_outofrange!\n");
		return;
	}
	
	if(place < place_now2)
	{
		place_det = place_now2 - place;
	}
	else
	{
		place_det = place_now2 - place;
	}
	
	if(place_det > 0)
	{
		if(place_det < 6)
		{
			load2_last_nflag(place_det);
			place_now2 = place;
		}
		else
		{
			place_det = SCORE_NUM2 - place_det;
			load2_next_nflag(place_det);
			place_now2 = place;
		}

	}
	else if(place_det < 0)
	{
		if(place_det > -6)
		{
			load2_next_nflag(-place_det);
			place_now2 = place;
		}
		else
		{
			place_det = -SCORE_NUM2 - place_det;
			load2_last_nflag(-place_det);
			place_now2 = place;
		}
	}
}

void load2_scores_in(uint8_t id)
{
//	sprintf(printfbuff,"scorcnt:%d,inid:%#x\n",scorecnt2,id);
//	Serial_Send(&huart1,printfbuff);
//	memset(printfbuff,0,sizeof(printfbuff));
	//printf("id:%d\n",id);
	
	if(scorecnt2 == place_now2)
	{
		scoreid2[scorecnt2] = id;
//		printf("score_in!");
		if(scorecnt2 < SCORE_NUM2-1)
		{
			scorecnt2++;
			place_now2++;
		}
		load2_next_flag();
	}
	else
	{
		scoreid2[place_now2] = id;
		load2_to_place(scorecnt2);
	}
}

uint8_t load2_scores_getcnt(void)
{
	return scorecnt2;
}

void load2_scores_outid(uint8_t id)
{
	ball_outid2 = id;
	load2_sta = 2;
}

void load2_scores_out(uint8_t id)
{
	target_place2 = SCORE_NUM2+3;
//	printf("in");
	for(uint8_t i = 0;i < SCORE_NUM2;i++)
	{
		if(scoreid2[i] == id)
		{
			target_place2 = i;
			break;
		}
	}
	
	if(target_place2 > SCORE_NUM2-1)
	{
		//sprintf(printfbuff,"notball!\n");
		//Serial_Send(&huart1,printfbuff);
		memset(printfbuff,0,sizeof(printfbuff));
		//ERROR!! not the id ball in load!!!
	}
	else
	{
		load2_to_place(target_place2);
//		load2_sta = 0;
	}
}

uint8_t temple_sort[SCORE_NUM2] = {0x11,0x12,0x13,0x21,0x22,0x23,0x31,0x32,0x33}; 
uint8_t load_error_flag = 0;

void load_check2(void)
{
	uint8_t load_copy[SCORE_NUM2];
    uint8_t sort_temp;
	
	for(uint8_t n = 0;n < SCORE_NUM2;n++)
	{
		load_copy[n] = scoreid2[n];
	}
	
    //冒泡排序算法：进行 n-1 轮比较
    for(uint8_t i=0; i<SCORE_NUM2-1; i++){
        //每一轮比较前 n-1-i 个，也就是说，已经排序好的最后 i 个不用比较
        for(uint8_t j=0; j<SCORE_NUM2-1-i; j++){
            if(load_copy[j] > load_copy[j+1]){
                sort_temp = load_copy[j];
                load_copy[j] = load_copy[j+1];
                load_copy[j+1] = sort_temp;
            }
        }
    }
	
	for(uint8_t i = 0;i < SCORE_NUM2;i++)
	{
		if(load_copy[i] != temple_sort[i])
		{
			load_error_flag = 1;
			break;
		}
	}
	
	if(load_error_flag == 1)
	{
		
	}
	
}
uint8_t error_place = 0;
	

void load_find_error(void)
{
	
}

