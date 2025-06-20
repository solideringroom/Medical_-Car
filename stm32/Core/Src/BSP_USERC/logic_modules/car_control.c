
	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    上层封装，控制车模型的前进后退左右转
  * @brief   
  ******************************************************************************
  * @attention	控制车按照给定速度走给定圈数，完成前进，左转，右转，后退
  * 
  * 
  *		
  * 
  * 使用：先定义一个cartype结构体对象，把它传入car_init()进行初始化，这里面包含了电机驱动初始化，编码器初始化，还有三个pid的初始化（包括使能）
  * 参数统一在car_params_config里面调
	*
	* 
  ******************************************************************************
  */
#include "car_control.h"

uint8_t RESET_DISTANCE_FLAG = 0;
//这些全局变量便于调试
float Left_Motor_Position_PID_Out,Left_Motor_Speed_PID_Out;
float Right_Motor_Position_PID_Out,Right_Motor_Speed_PID_Out;
float COMPO_ER_OUT;			//补偿环PID的输出
Filter_Average ERROR_FILTER;//这个滤波器单独给双轮之前误差用
positional_pid_ctrler ERROR_DISTANCE_PID;//左右轮之间的误差环
positional_pid_ctrler FIND_LINE_PID;//巡线环pid
int Find_Line_Error;//巡线环误差，方便调试
float Find_Line_LOUT,Find_Line_ROUT;
float Find_Line_EROutput;
//Filter_Average FindLine_FILTER;//这个滤波器单独给巡线差速环用(好像用不到，PID结构体里面有自己的filter)
#define ABS(x) ((x > 0) ? x : -x)

void ERROR_EARSER_INIT()
{
	positional_pid_controller_init(&ERROR_DISTANCE_PID,ERROR_KP_DISTANCE,ERROR_KI_DISTANCE,ERROR_KD_DISTANCE,ERROR_DeadZone_DISTANCE,ERROR_Separation_Threshold_DISTANCE,ERROR_Integral_Limit_Max_DISTANCE,ERROR_Integral_Limit_Min_DISTANCE,ERROR_OutPut_Max_DISTANCE,ERROR_OutPut_Min_DISTANCE);
	positional_pid_control(&ERROR_DISTANCE_PID,PID_ENABLE);
}
//后面位置环开关要用
void ERROR_EARSER_OFF()
{
	positional_pid_control(&ERROR_DISTANCE_PID,PID_DISABLE);
}
void ERROR_EARSER_ON()
{
	positional_pid_control(&ERROR_DISTANCE_PID,PID_ENABLE);
}
float EARSE_M1_M2_ERROR(uint8_t stright_or_turn)
{
	if(stright_or_turn == 0)
	{
		COMPO_ER_OUT = positional_pid_compute(&ERROR_DISTANCE_PID,Get_M1_Distance(),Get_M2_Distance(),&ERROR_FILTER);
	}
	else
	{
		COMPO_ER_OUT = positional_pid_compute(&ERROR_DISTANCE_PID,ABS(Get_M1_Distance()),ABS(Get_M2_Distance()),&ERROR_FILTER);
	}
	//COMPO_ER_OUT = 0.003*error_distance;
	return COMPO_ER_OUT;
}
//*********************动作函数结束之后返回0说明没跑完，返回1说明该动作结束。用于状态机，判断是否切换下一个状态
//*********************关于一个动作结束之后是否要关闭PID，如果关了那么速度环的积分就没了，而且抗干扰能力减弱，过冲的化回不来。
//*********************注意下面动作函数里面的速度和位移单位都是圈，不是编码器的脉冲
uint8_t Go_Forward(CarType* Car,float speed_rev,float distance,float DEAD_ZONE)
{
	Left_DeadZone_DISTANCE = DEAD_ZONE;
	Right_DeadZone_DISTANCE = DEAD_ZONE;
	float speed = speed_rev;	
	//单次执行清空以前路程，直到控制结束
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//速度环PI控制器，位置环PD控制器
	//速度的控制方式：修改位置环的输出限幅，因为开始的时候位置环让它饱和，调大P来减小稳态误差，D防止其过冲
	//修改左，右边电机位置环的输出限幅，达到控制其速度的目的
	Car->PID_Left_Motor_Distance.output_max = speed;
	Car->PID_Left_Motor_Distance.output_min = -speed;
	
	Car->PID_Right_Motor_Distance.output_max = speed;
	Car->PID_Right_Motor_Distance.output_min = -speed;
	//计算左，右电机位置环PID输出，传入到速度环，最后算出速度环结果
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(0),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(0),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//把计算结果加载到PWM输出
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//如果左右轮子位置误差进入死区，说明很接近目标了，直接停止，把清空里程标志位恢复,把PID关闭
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(1,200))
		{
			RESET_DISTANCE_FLAG = 0;
			ERROR_DISTANCE_PID.i_out = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Go_Backward(CarType* Car,float speed_rev,float distance,float DEAD_ZONE)
{
	Left_DeadZone_DISTANCE = DEAD_ZONE;
	Right_DeadZone_DISTANCE = DEAD_ZONE;
	//反转速度，路程，因为要向后走
	float speed = speed_rev;
	speed = -speed;
	distance = -distance;
	
	//单次执行清空以前路程，直到控制结束
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//速度环PI控制器，位置环PD控制器
	//速度的控制方式：修改位置环的输出限幅，因为开始的时候位置环让它饱和，调大P来减小稳态误差，D防止其过冲
	//修改左，右边电机位置环的输出限幅，达到控制其速度的目的
	Car->PID_Left_Motor_Distance.output_max = -speed;
	Car->PID_Left_Motor_Distance.output_min = speed;
	
	Car->PID_Right_Motor_Distance.output_max = -speed;
	Car->PID_Right_Motor_Distance.output_min = speed;
	//计算左，右电机位置环PID输出，传入到速度环，最后算出速度环结果
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(0),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(0),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//把计算结果加载到PWM输出
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//如果左右轮子位置误差进入死区，说明很接近目标了，直接停止，把清空里程标志位恢复
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
				if(delay_timer(2,200))
		{
			RESET_DISTANCE_FLAG = 0;
			ERROR_DISTANCE_PID.i_out = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Turn_Left(CarType* Car,float speed_rev,float distance,float DEAD_ZONE)
{
	Left_DeadZone_DISTANCE = DEAD_ZONE;
	Right_DeadZone_DISTANCE = DEAD_ZONE;
	//反转速度，路程，因为要向后走
	float speed = speed_rev;
	float speed_left = -speed;
	float speed_right = speed;
	float distance_left = -distance;
	float distance_right = distance;
	
	//单次执行清空以前路程，直到控制结束
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//速度环PI控制器，位置环PD控制器
	//速度的控制方式：修改位置环的输出限幅，因为开始的时候位置环让它饱和，调大P来减小稳态误差，D防止其过冲
	//修改左，右边电机位置环的输出限幅，达到控制其速度的目的
	Car->PID_Left_Motor_Distance.output_max = -speed_left;
	Car->PID_Left_Motor_Distance.output_min = speed_left;
	
	Car->PID_Right_Motor_Distance.output_max = speed_right;
	Car->PID_Right_Motor_Distance.output_min = -speed_right;
	//计算左，右电机位置环PID输出，传入到速度环，最后算出速度环结果
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance_left,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(1),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance_right,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out+EARSE_M1_M2_ERROR(1),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//把计算结果加载到PWM输出
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//如果左右轮子位置误差进入死区，说明很接近目标了，直接停止，把清空里程标志位恢复
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(3,200))
		{
			RESET_DISTANCE_FLAG = 0;
			ERROR_DISTANCE_PID.i_out = 0;
			return 1;
		}
	}
	return 0;
}

uint8_t Turn_Right(CarType* Car,float speed_rev,float distance,float DEAD_ZONE)
{
	Left_DeadZone_DISTANCE = DEAD_ZONE;
	Right_DeadZone_DISTANCE = DEAD_ZONE;
	//反转速度，路程，因为要向后走
	float speed = speed_rev;
	float speed_left = -speed;
	float speed_right = speed;
	float distance_left = distance;
	float distance_right = -distance;
	
	//单次执行清空以前路程，直到控制结束
	if(RESET_DISTANCE_FLAG == 0)
	{
		Reset_M1_Distance();
		Reset_M2_Distance();
		RESET_DISTANCE_FLAG = 1;
	}
	//速度环PI控制器，位置环PD控制器
	//速度的控制方式：修改位置环的输出限幅，因为开始的时候位置环让它饱和，调大P来减小稳态误差，D防止其过冲
	//修改左，右边电机位置环的输出限幅，达到控制其速度的目的
	Car->PID_Left_Motor_Distance.output_max = -speed_left;
	Car->PID_Left_Motor_Distance.output_min = speed_left;
	
	Car->PID_Right_Motor_Distance.output_max = speed_right;
	Car->PID_Right_Motor_Distance.output_min = -speed_right;
	//计算左，右电机位置环PID输出，传入到速度环，最后算出速度环结果
	Left_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Distance,distance_left,Get_M1_Distance(),&Car->PID_Left_Motor_Distance.FILTER_ERROR);
	Left_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Left_Motor_Speed,Left_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(1),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
	
	Right_Motor_Position_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Distance,distance_right,Get_M2_Distance(),&Car->PID_Right_Motor_Distance.FILTER_ERROR);
	Right_Motor_Speed_PID_Out = positional_pid_compute(&Car->PID_Right_Motor_Speed,Right_Motor_Position_PID_Out-EARSE_M1_M2_ERROR(1),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
	
	//把计算结果加载到PWM输出
	Load(Left_Motor_Speed_PID_Out,Right_Motor_Speed_PID_Out);
	
	//如果左右轮子位置误差进入死区，说明很接近目标了，直接停止，把清空里程标志位恢复
	if(Left_Motor_Position_PID_Out == 0 && Right_Motor_Position_PID_Out ==0)
	{
		Load(0,0);
		if(delay_timer(4,200))
		{
			RESET_DISTANCE_FLAG = 0;
			ERROR_DISTANCE_PID.i_out = 0;
			return 1;
		}
	}
	return 0;
}


//里面的PID变量在car_params_config.c里面配置
uint8_t Car_Init(CarType *Car)
{
	positional_pid_controller_init(&Car->PID_Left_Motor_Speed,Left_KP_SPEED,Left_KI_SPEED,Left_KD_SPEED,Left_DeadZone_SPEED,Left_Separation_Threshold_SPEED,Left_Integral_Limit_Max_SPEED,Left_Integral_Limit_Min_SPEED,Left_OutPut_Max_SPEED,Left_OutPut_Min_SPEED);
	positional_pid_controller_init(&Car->PID_Right_Motor_Speed,Right_KP_SPEED,Right_KI_SPEED,Right_KD_SPEED,Right_DeadZone_SPEED,Right_Separation_Threshold_SPEED,Right_Integral_Limit_Max_SPEED,Right_Integral_Limit_Min_SPEED,Right_OutPut_Max_SPEED,Right_OutPut_Min_SPEED);
	positional_pid_controller_init(&Car->PID_Left_Motor_Distance,Left_KP_DISTANCE,Left_KI_DISTANCE,Left_KD_DISTANCE,Left_DeadZone_DISTANCE,Left_Separation_Threshold_DISTANCE,Left_Integral_Limit_Max_DISTANCE,Left_Integral_Limit_Min_DISTANCE,Left_OutPut_Max_DISTANCE,Left_OutPut_Min_DISTANCE);
	positional_pid_controller_init(&Car->PID_Right_Motor_Distance,Right_KP_DISTANCE,Right_KI_DISTANCE,Right_KD_DISTANCE,Right_DeadZone_DISTANCE,Right_Separation_Threshold_DISTANCE,Right_Integral_Limit_Max_DISTANCE,Right_Integral_Limit_Min_DISTANCE,Right_OutPut_Max_DISTANCE,Right_OutPut_Min_DISTANCE);

	Encoder_Start();
	TB6612_Init();
	ERROR_EARSER_INIT();
	positional_pid_controller_init(&FIND_LINE_PID,FINDLINE_KP_DISTANCE,FINDLINE_KI_DISTANCE,FINDLINE_KD_DISTANCE,FINDLINE_DeadZone_DISTANCE,FINDLINE_Separation_Threshold_DISTANCE,FINDLINE_Integral_Limit_Max_DISTANCE,FINDLINE_Integral_Limit_Min_DISTANCE,FINDLINE_OutPut_Max_DISTANCE,FINDLINE_OutPut_Min_DISTANCE);
	return 1;
}

void Car_Position_Mode_ON(CarType *Car)
{
	//开启两个电机的PID（双环）
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_ENABLE);
	positional_pid_control(&Car->PID_Left_Motor_Distance,PID_ENABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_ENABLE);
	positional_pid_control(&Car->PID_Right_Motor_Distance,PID_ENABLE);
	ERROR_EARSER_ON();
}
void Car_Position_Mode_OFF(CarType *Car)
{
	//关闭两个电机的PID（双环）
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_DISABLE);
	positional_pid_control(&Car->PID_Left_Motor_Distance,PID_DISABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_DISABLE);
	positional_pid_control(&Car->PID_Right_Motor_Distance,PID_DISABLE);
	ERROR_EARSER_OFF();
	Load(0,0);
}	
void Car_FindLine_Mode_ON(CarType *Car)
{
	//开启两个电机的PID（双环）
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_ENABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_ENABLE);
	positional_pid_control(&FIND_LINE_PID,PID_ENABLE);
	
}
void Car_FindLine_Mode_OFF(CarType *Car)
{
	//开启两个电机的PID（双环）
	positional_pid_control(&Car->PID_Left_Motor_Speed,PID_DISABLE);
	positional_pid_control(&Car->PID_Right_Motor_Speed,PID_DISABLE);
	positional_pid_control(&FIND_LINE_PID,PID_DISABLE);
	Load(0,0);
}
//Mode 1:寻中线 0：回正，Speed为0
//pack是视觉发过来的数据解析之后的数据包，有中线距离的偏移量[0]，中线的偏移角度量[1]
//左负右正，左偏的时候左边电机需要加速，所以减去error，右边加上error
uint8_t Car_Find_Line(CarType* Car,float Base_Speed,uint8_t Mode,float Mode_Gain,int* pack,float Over_All_Gain) //Mode指的是回正还是寻中线 Mode Gain指的是如果是回正模式的话PID误差成比例减小，那么增大其输出
{
	if(1 == Mode) //寻中线模式
	{

			if(pack[2] == 0) //对应其他情况，可以直行
			{
				Find_Line_Error = pack[0]+pack[1];
				Find_Line_EROutput = positional_pid_compute(&FIND_LINE_PID,0,Find_Line_Error,&FIND_LINE_PID.FILTER_ERROR);
				Find_Line_LOUT = positional_pid_compute(&Car->PID_Left_Motor_Speed,Over_All_Gain*(Base_Speed+Find_Line_EROutput),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
				Find_Line_ROUT = positional_pid_compute(&Car->PID_Right_Motor_Speed,Over_All_Gain*(Base_Speed-Find_Line_EROutput),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
			}
			else if(pack[2] == 1)
			{ //识别到右手十字路口
					return 1;
			}
			else if(pack[2] == 2)
			{ //识别到左手十字路口
					return 2;
			}
			else if(pack[2] == 3)
			{ //识别到十字路口
					return 3;
			}
			else if(pack[2] == 4)
			{ //到药房了
					return 4;
			}
			Load(Find_Line_LOUT,Find_Line_ROUT);

		
	}else if(0 == Mode)
	{
//		if(pack[0] == -1 && pack[1] == -1){ //收到-1-1表明小车完全未识别到线,原地转圈找线
//			Find_Line_LOUT = positional_pid_compute(&Car->PID_Left_Motor_Speed, 0.5, Get_M1_Speed(), &Car->PID_Left_Motor_Speed.FILTER_ERROR);
//			Find_Line_ROUT = positional_pid_compute(&Car->PID_Right_Motor_Speed, -0.5,Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
//		}
//		else{
			Find_Line_Error = Mode_Gain*pack[1];
			Find_Line_EROutput = positional_pid_compute(&FIND_LINE_PID,0,Find_Line_Error,&FIND_LINE_PID.FILTER_ERROR);
			Find_Line_LOUT = positional_pid_compute(&Car->PID_Left_Motor_Speed,Over_All_Gain*(Find_Line_EROutput),Get_M1_Speed(),&Car->PID_Left_Motor_Speed.FILTER_ERROR);
			Find_Line_ROUT = positional_pid_compute(&Car->PID_Right_Motor_Speed,Over_All_Gain*(-Find_Line_EROutput),Get_M2_Speed(),&Car->PID_Right_Motor_Speed.FILTER_ERROR);
			Load(Find_Line_LOUT,Find_Line_ROUT);
			if(pack[1]<2&&pack[1]>-2)
			{
				return 5;//进入死区，表示对齐了
			}
//		}
		
	}
	return 0;
}