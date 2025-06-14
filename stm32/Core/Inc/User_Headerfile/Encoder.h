#ifndef _ENCODER_H
#define _ENCODER_H
#define ONE_REV 1760//11*40*4，可以根据实际更改
#define FILTER_LEVEL 7  //滤波等级，越高电机控制越稳，但是计算成本高
#include "main.h"

typedef struct
{
	float speed_buffer[FILTER_LEVEL]; // FILTER_LEVEL次测速值
	int buffer_idx;          // 数组索引
	uint8_t flag;
}	Filter_Average;

void Encoder_Start(void);//初始化

void Calc_Speed();//给定时器6中断用的
//单位都是转/秒
float Get_M1_Speed();
float Get_M2_Speed();
//因为距离是相对距离，这个函数重新定义距离0
void Reset_M1_Distance();
void Reset_M2_Distance();
//获取相对距离，单位是转
float Get_M1_Distance();
float Get_M2_Distance();

float get_filtered_speed(Filter_Average* Filter_OPP,float raw_speed) ;
//打开相应的filter
extern Filter_Average SPEED_FILTER_LEFT;
extern Filter_Average SPEED_FILTER_RIGHT;
extern float M1_SPEED_REC,M2_SPEED_REC;
extern float M1_DISTANCE_REC,M2_DISTANCE_REC;
#endif
