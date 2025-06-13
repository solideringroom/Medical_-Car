#include "Encoder.h"
#include "main.h"
#include "car_control.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;

short temp_new_m1,temp_old_m1,speed_pluse_m1;//�洢�����һ�ζ�ʱ�����ڵļ�������������ֵ�����ڼ����ٶ� 
short temp_new_m2,temp_old_m2,speed_pluse_m2;

short m1_distance_pulse,m2_distance_pulse;
short m1_distance_rev,m2_distance_rev;

float M1_SPEED_REC,M2_SPEED_REC;
float M1_DISTANCE_REC,M2_DISTANCE_REC;


Filter_Average SPEED_FILTER_LEFT;
Filter_Average SPEED_FILTER_RIGHT;

float get_filtered_speed(Filter_Average* Filter_OPP,float raw_speed) {
	if(Filter_OPP->flag == 0)
	{
		for (int i=0; i<FILTER_LEVEL; i++) Filter_OPP->speed_buffer[i] = raw_speed;
		Filter_OPP->flag = 1;//��һ�β����£�֮���ٸ���
	}
    Filter_OPP->speed_buffer[Filter_OPP->buffer_idx] = raw_speed;
		Filter_OPP->buffer_idx = (Filter_OPP->buffer_idx + 1)%FILTER_LEVEL;
    float sum = 0;
    for (int i=0; i<FILTER_LEVEL; i++) sum += Filter_OPP->speed_buffer[i];
    return (float)sum / FILTER_LEVEL; // FILTER_LEVEL��ƽ��
}

void Encoder_Start(void)
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	Reset_M1_Distance();
	Reset_M2_Distance();
	HAL_TIM_Base_Start_IT(&htim6);
}

//TIM4��M1�ı�������TIM8��M2�ı�����

void Calc_Speed()//����TIM6�ж��ִ��Ƶ��Ϊ50HZ
{
	//����M1�ٶ�
	temp_new_m1 = LEFT_MOTOR_POLE*(short)__HAL_TIM_GetCounter(&htim4);//ǿ��ת����short���ͣ���Ϊshort��-32768--32767
	speed_pluse_m1 = (temp_new_m1 - temp_old_m1)*100;//��λ������/�룬֮���ȡ����ת����ת/��
	temp_old_m1 = temp_new_m1;
	//����M1����
	if(temp_new_m1>=ONE_REV)
	{
		m1_distance_rev+=1;
		__HAL_TIM_SetCounter(&htim4,0);//��ֹ���
		temp_old_m1 = 0;//��ֹ�ٶ�ͻ�� 
	}else if(temp_new_m1<=-ONE_REV)
	{
		m1_distance_rev-=1;
		__HAL_TIM_SetCounter(&htim4,0);//��ֹ���
		temp_old_m1 = 0;//��ֹ�ٶ�ͻ��
	}
	//*********************************************************************************************************���洦��M2
	//����M2�ٶ�
	temp_new_m2 = ROGHT_MOTOR_POLE*(short)__HAL_TIM_GetCounter(&htim8);//ǿ��ת����short���ͣ���Ϊshort��-32768--32767
	speed_pluse_m2 = (temp_new_m2 - temp_old_m2)*100;//��λ������/�룬֮���ȡ����ת����ת/��
	temp_old_m2 = temp_new_m2;
	//����M2����
	if(temp_new_m2>=ONE_REV)
	{
		m2_distance_rev+=1;
		__HAL_TIM_SetCounter(&htim8,0);//��ֹ���
		temp_old_m2 = 0;//��ֹ�ٶ�ͻ��
	}else if(temp_new_m2<=-ONE_REV)
	{
		m2_distance_rev-=1;
		__HAL_TIM_SetCounter(&htim8,0);//��ֹ���
		temp_old_m2 = 0;//��ֹ�ٶ�ͻ��
	}
}

float Get_M1_Speed()
{
	M1_SPEED_REC = get_filtered_speed(&SPEED_FILTER_LEFT,(float)speed_pluse_m1/ONE_REV);
	return M1_SPEED_REC;//��λ��Ȧÿ�룬float����
}

float Get_M2_Speed()
{
	M2_SPEED_REC = get_filtered_speed(&SPEED_FILTER_RIGHT,(float)speed_pluse_m2/ONE_REV);
	return  M2_SPEED_REC;//��λ��Ȧÿ�룬float����
}

void Reset_M1_Distance()
{
	__HAL_TIM_SetCounter(&htim4,0);//����
	temp_old_m1 = 0;//��ֹ�ٶ�ͻ��
	m1_distance_rev = 0;
	return ;
}
void Reset_M2_Distance()
{
	__HAL_TIM_SetCounter(&htim8,0);//����
	temp_old_m2 = 0;//��ֹ�ٶ�ͻ��
	m2_distance_rev = 0;
	return ;
}

float Get_M1_Distance()
{
	M1_DISTANCE_REC = (float)m1_distance_rev + (float)LEFT_MOTOR_POLE*((short)__HAL_TIM_GetCounter(&htim4))/ONE_REV;
	return M1_DISTANCE_REC;
}

float Get_M2_Distance()
{
	M2_DISTANCE_REC = (float)m2_distance_rev + (float)ROGHT_MOTOR_POLE*((short)__HAL_TIM_GetCounter(&htim8))/ONE_REV;
	return M2_DISTANCE_REC;
}












	