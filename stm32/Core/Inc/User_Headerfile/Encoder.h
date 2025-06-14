#ifndef _ENCODER_H
#define _ENCODER_H
#define ONE_REV 1760//11*40*4�����Ը���ʵ�ʸ���
#define FILTER_LEVEL 7  //�˲��ȼ���Խ�ߵ������Խ�ȣ����Ǽ���ɱ���
#include "main.h"

typedef struct
{
	float speed_buffer[FILTER_LEVEL]; // FILTER_LEVEL�β���ֵ
	int buffer_idx;          // ��������
	uint8_t flag;
}	Filter_Average;

void Encoder_Start(void);//��ʼ��

void Calc_Speed();//����ʱ��6�ж��õ�
//��λ����ת/��
float Get_M1_Speed();
float Get_M2_Speed();
//��Ϊ��������Ծ��룬����������¶������0
void Reset_M1_Distance();
void Reset_M2_Distance();
//��ȡ��Ծ��룬��λ��ת
float Get_M1_Distance();
float Get_M2_Distance();

float get_filtered_speed(Filter_Average* Filter_OPP,float raw_speed) ;
//����Ӧ��filter
extern Filter_Average SPEED_FILTER_LEFT;
extern Filter_Average SPEED_FILTER_RIGHT;
extern float M1_SPEED_REC,M2_SPEED_REC;
extern float M1_DISTANCE_REC,M2_DISTANCE_REC;
#endif
