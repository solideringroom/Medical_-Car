
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   送药小车的状态机
  * @brief   
  ******************************************************************************
  * @attention	
  * 		先粗略实现了前进，左转，右转，后退回到原点。
				每一个动作必须先完成（每个动作函数完成之后会返回一个标志），才能更新标志位，执行下一个任务
  * 		
  *			确定大任务，再拆解成小任务
  * 
  * 
  * 
	*
	* 
  ******************************************************************************
  */
#include "Task_Machine.h"
#include "car_control.h"
#include "MV_UART.h"
// 具体位置信息分配如下所示，请对照实际地图理解
/*
           f                                       g
         ------                                 ------
           |                                       |
           |                                       |
           |                                       |
           |                                       |
           |4__________________3__________________5|
           |                   |                   |
           |                   |                   |
           |                   |                   |
           |                   |                   |
         __|___                |                 __|___
           e                   |                   h
                               |
                               |
                     |         |          |
                    c|_________2__________|d
                     |         |          |
                               |
                               |
                               |
                               |
                               |
                               |
                               |
                     |         |          |
                    a|.........1..........|b
                     |         |          |
                               |
                               |
                               |
                               |
                               |
                               |
                            .......
                               0
*/

TASK_STATUS Medicine_Car_Task;
uint8_t a_temp_step ;
uint8_t flip_flag0;
//
void TASK_RUN(CarType* Task_Car)
{
	//任务1，把药送到近端病房
	switch (Medicine_Car_Task)
	{
		case APP_CAR_CONTROL_INIT:
			if(1 == Car_Init(Task_Car))
			{
				Medicine_Car_Task++;				//第一步初始化成功
			}
		
			break;
		case APP_CAR_CONTROL_WAIT_FOR_LOAD: //药品装载完毕
			if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
			{
				Medicine_Car_Task++;
			}
			break;
		case APP_CAR_CONTROL_ESTABLISH_COMMUNICATION:  //任务发送给视觉了
			if( 1 == Transmit(&T_Data) || flip_flag0 == 1 )
			{
				flip_flag0 = 1;
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
				if(delay_timer(DELAY_TASK_0,1000))
				{
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
					flip_flag0 = 0;
					Medicine_Car_Task++;
				}
			}
			break;
		case APP_CAR_CONTROL_FIND_RED_LINE:    //测试巡线用
			if(1)
			{
				Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0);//这里卡死，先一直运行
			}
			break;
		case APP_CAR_CONTROL_GOTO_A:
			switch (a_temp_step)
			{
				case 0:
					if(1 == Go_Forward(Task_Car,1,8))								//前进
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;																	
					}
					break;
				case 1:
					if(1 == Turn_Left(Task_Car,0.8,0.5))							//左转
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Right(Task_Car,0.8,0.5))							//右转
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;												
					}
					break;
				case 3:
					if(1 == Go_Backward(Task_Car,1,8))							//后退
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;														
					}
					break;
				case 4:
					a_temp_step++;																		//结束，任务进入结束停滞区
					Medicine_Car_Task++;
					break;
				default:
					break;	
			}
			break;
		
		case APP_CAR_CONTROL_FLASH:
				HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_3);		//点灯庆祝
				Medicine_Car_Task++;
				Car_Position_Mode_OFF(Task_Car);								//关闭所有PID输出
				
				break;
		case APP_CAR_CONTROL_STOP:
				Car_Position_Mode_OFF(Task_Car);		
				break;		
		default:
			break;																		//结束之后所有任务标志位全处于结束停滞区，不复位就不会再次执行任何东西
	}
}





