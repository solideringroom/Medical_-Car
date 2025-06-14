
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    电机驱动
  * @brief   
  ******************************************************************************
  * @attention	
  * PWM_MAX/MIN要根据自己开的时钟频率来算。我用的f4，直接把频率拉满了。
  * 
  *		
  * 
  * 使用：使用的时候先调用初始化函数
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
//之后根据实际情况调整极性
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
  * 控制频率10khz
  * @param m1 speed,m2 speed 
  * @retval None
  */


//首先初始化产生pwm的定时器
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


