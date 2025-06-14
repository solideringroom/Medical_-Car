
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    �������
  * @brief   
  ******************************************************************************
  * @attention	
  * PWM_MAX/MINҪ�����Լ�����ʱ��Ƶ�����㡣���õ�f4��ֱ�Ӱ�Ƶ�������ˡ�
  * 
  *		
  * 
  * ʹ�ã�ʹ�õ�ʱ���ȵ��ó�ʼ������
  * 
	*
	* 
  ******************************************************************************
  */

#include "TB6612.h"
#include "main.h"

#define PWM_MAX 8400	
#define PWM_MIN -8400

extern TIM_HandleTypeDef htim2;


int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}
//֮�����ʵ�������������
//M1:
//PE7       AIN1
//PC0      	AIN2
//PWM1:     TIM2CH2
//M2:
//PE1       BIN1 
//PE0       BIN2
//PWM2:			TIM2CH3

/**
  * @brief Motor Drive
  * ����Ƶ��10khz
  * @param m1 speed,m2 speed 
  * @retval None
  */


//���ȳ�ʼ������pwm�Ķ�ʱ��
void TB6612_Init()
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
}

void Load(int moto1,int moto2)			//-8400~8400
{
	if(moto1>PWM_MAX) moto1=PWM_MAX;
	if(moto1<PWM_MIN) moto1=PWM_MIN;
	if(moto2>PWM_MAX) moto2=PWM_MAX;
	if(moto2<PWM_MIN) moto2=PWM_MIN;
	//
	if(moto1>0)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,abs(moto1));
	if(moto2>0)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,abs(moto2));
}


