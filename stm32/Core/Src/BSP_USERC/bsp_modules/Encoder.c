/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    编码器测速
  * @brief   
  ******************************************************************************
  * @attention	
  * 开启定时器4，8来读取编码器
  * 定时器6是用于开启100HZ频率（不是100HZ要改程序）的定时器中断，里面需要自行把Calc_Speed()函数
  *	写中断来计算M1,M2的速度和路程
  *	
  * 数据读取和控制的函数可以一并放在TIM6里面
  * 使用：记得先把car_params_config.h搞进来，先调用初始化函数，之后把计算速度距离的函数放到Timer6中断里面
  * 调试：直接查看M1,M2 SPEED和DISTANCE
	*
	* 使用不同编码器或电机时在.h里面调节轮子转一圈产生的脉冲数
  ******************************************************************************
  */
#include "car_params_config.h"
#include "Encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;

//注意这里short是为了有正负值
short temp_new_m1,temp_old_m1,speed_pluse_m1;//存储间隔了一次定时器周期的计数器的两个数值，用于计算速度 
short temp_new_m2,temp_old_m2,speed_pluse_m2;

//short m1_distance_pulse,m2_distance_pulse;好像没用
short m1_distance_rev,m2_distance_rev;

float M1_SPEED_REC,M2_SPEED_REC;						 //返回的是M1M2的速度，主要能方便调试用
float M1_DISTANCE_REC,M2_DISTANCE_REC;			 //同理，返回M1,M2距离，方便调试

//两个滤波器结构体对象，这样搞主要是每次声明每个滤波器都能有单独的缓存空间
//这样个滤波器主要给左右轮子的速度采样进行平均值滤波
Filter_Average SPEED_FILTER_LEFT;
Filter_Average SPEED_FILTER_RIGHT;
//滤波函数
float get_filtered_speed(Filter_Average* Filter_OPP,float raw_speed) {
	if(Filter_OPP->flag == 0)
	{
		for (int i=0; i<FILTER_LEVEL; i++) Filter_OPP->speed_buffer[i] = raw_speed;
		Filter_OPP->flag = 1;//第一次不出事，之后再覆盖
	}
    Filter_OPP->speed_buffer[Filter_OPP->buffer_idx] = raw_speed;
		Filter_OPP->buffer_idx = (Filter_OPP->buffer_idx + 1)%FILTER_LEVEL;
    float sum = 0;
    for (int i=0; i<FILTER_LEVEL; i++) sum += Filter_OPP->speed_buffer[i];
    return (float)sum / FILTER_LEVEL; // FILTER_LEVEL次平均
}
//编码器初始化，主要是重置M1,M2路程，把编码器打开，把定时器6的中断打开
void Encoder_Start(void)
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
	Reset_M1_Distance();
	Reset_M2_Distance();
	HAL_TIM_Base_Start_IT(&htim6);
}

//TIM4是M1的编码器，TIM8是M2的编码器

void Calc_Speed()//放在TIM6中断里，执行频率为100HZ
{
	//计算M1速度
	temp_new_m1 = LEFT_MOTOR_POLE*(short)__HAL_TIM_GetCounter(&htim4);//强制转化成short类型，因为short是-32768--32767
	speed_pluse_m1 = (temp_new_m1 - temp_old_m1)*100;//单位是脉冲/秒，之后读取可以转化成转/秒
	temp_old_m1 = temp_new_m1;
	//计算M1距离
	if(temp_new_m1>=ONE_REV)
	{
		m1_distance_rev+=1;
		__HAL_TIM_SetCounter(&htim4,0);//防止溢出
		temp_old_m1 = 0;//防止速度突变 
	}else if(temp_new_m1<=-ONE_REV)
	{
		m1_distance_rev-=1;
		__HAL_TIM_SetCounter(&htim4,0);//防止溢出
		temp_old_m1 = 0;//防止速度突变
	}
	//*********************************************************************************************************下面处理M2
	//计算M2速度
	temp_new_m2 = ROGHT_MOTOR_POLE*(short)__HAL_TIM_GetCounter(&htim8);//强制转化成short类型，因为short是-32768--32767
	speed_pluse_m2 = (temp_new_m2 - temp_old_m2)*100;//单位是脉冲/秒，之后读取可以转化成转/秒
	temp_old_m2 = temp_new_m2;
	//计算M2距离
	if(temp_new_m2>=ONE_REV)
	{
		m2_distance_rev+=1;
		__HAL_TIM_SetCounter(&htim8,0);//防止溢出
		temp_old_m2 = 0;//防止速度突变
	}else if(temp_new_m2<=-ONE_REV)
	{
		m2_distance_rev-=1;
		__HAL_TIM_SetCounter(&htim8,0);//防止溢出
		temp_old_m2 = 0;//防止速度突变
	}
}

float Get_M1_Speed()
{
	M1_SPEED_REC = get_filtered_speed(&SPEED_FILTER_LEFT,(float)speed_pluse_m1/ONE_REV);
	return M1_SPEED_REC;//单位是圈每秒，float类型
}

float Get_M2_Speed()
{
	M2_SPEED_REC = get_filtered_speed(&SPEED_FILTER_RIGHT,(float)speed_pluse_m2/ONE_REV);
	return  M2_SPEED_REC;//单位是圈每秒，float类型
}

void Reset_M1_Distance()
{
	__HAL_TIM_SetCounter(&htim4,0);//重置
	temp_old_m1 = 0;//防止速度突变
	m1_distance_rev = 0;
	return ;
}
void Reset_M2_Distance()
{
	__HAL_TIM_SetCounter(&htim8,0);//重置
	temp_old_m2 = 0;//防止速度突变
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












	