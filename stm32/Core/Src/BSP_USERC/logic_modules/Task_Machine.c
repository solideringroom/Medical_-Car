
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
uint8_t time_flag;
uint8_t flag_counter = 0;
TASK_STATUS Medicine_Car_Task;
uint8_t a_temp_step,d_temp_step,f_temp_step;

uint8_t flip_flag0;
slope_function speed_slop;
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
		case APP_CAR_CONTROL_WAIT_FOR_NUM:    //这一步开始的数字识别等待和视觉对接
			if(1)
			{
				Medicine_Car_Task = APP_CAR_CONTROL_GOTO_F;//********************************先测试一波远端病房
				Car_FindLine_Mode_ON(Task_Car);	//先打开循迹PID，准备循迹
			}
			break;
		case APP_CAR_CONTROL_GOTO_A:
			//a_temp_step = 9;//测试用
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,0.5,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,0.8,0.46,0.05))							//左转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,1000),1,0,Recv_Buff,1.0) == 4)					//巡线直到药房，之后停止
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 4:
					if(Go_Forward(Task_Car,0.5,0.2,0.02) == 1)					//	确保停止的瞬间偏转角度不会改变太大
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;
				case 5:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//等待卸载药品，之后回城
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 6:
					if(1 == Turn_Right(Task_Car,0.7,0.92,0.03))							//右转,其实是向后转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 7:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//巡线前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 8:
					
					if(1 == Go_Forward(Task_Car,0.5,0.75,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,0.8,0.5,0.05))							//右转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 10:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,999),1,0,Recv_Buff,1.0) == 4)								//巡线前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//任务序号进入死区								
					}
					break;
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_GOTO_D:
			//a_temp_step = 9;//测试用
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//前进
					{
						if(delay_timer(DELAY_TASK_5,1500)) //路口计数要冷却一段时间
						{
							time_flag = 0;
						}
						if(time_flag == 0)
						{
							flag_counter +=1;
							time_flag = 1;
						}
						if(flag_counter ==2) //第二个十字路口
						{
							Car_FindLine_Mode_OFF(Task_Car);
							Car_Position_Mode_ON(Task_Car);
							flag_counter = 0;
							a_temp_step++;
						}							
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,0.5,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,0.8,0.46,0.05))							//左转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,998),1,0,Recv_Buff,1.0) == 4)					//巡线直到药房，之后停止
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 4:
					if(Go_Forward(Task_Car,0.5,0.2,0.02) == 1)					//	确保停止的瞬间偏转角度不会改变太大
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;
				case 5:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//等待卸载药品，之后回城
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 6:
					if(1 == Turn_Right(Task_Car,0.7,0.92,0.02))							//右转,其实是向后转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 7:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//巡线前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 8:
					
					if(1 == Go_Forward(Task_Car,0.5,0.75,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,0.8,0.5,0.05))							//右转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 10://这个不清楚十字路口对巡线的干扰，实在不行遇到十字路口就直接打开位置环走过去再开巡线环
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,997),1,0,Recv_Buff,1.0) == 4)								//巡线前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//任务序号进入死区								
					}
					break;
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_GOTO_F:
			//a_temp_step = 9;//测试用
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,697),1,0,Recv_Buff,1.0) == 3)								//前进
					{
						if(delay_timer(DELAY_TASK_5,1000)) //路口计数要冷却一段时间
						{
							time_flag = 0;
						}
						if(time_flag == 0)
						{
							flag_counter +=1;
							time_flag = 1;
						}
						if(flag_counter == 3) //第三个十字路口
						{
							Car_FindLine_Mode_OFF(Task_Car);
							Car_Position_Mode_ON(Task_Car);
							flag_counter = 0;
							a_temp_step++;
						}							
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,1.5,0.46,0.05))							//左转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,596),1,0,Recv_Buff,1.0) == 3)					//巡线直到十字路口
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//然后要右转
				case 4:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 5://右转
					if(Turn_Right(Task_Car,1.5,0.46,0.05) == 1)					
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 6://
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,595),1,0,Recv_Buff,1.0) == 4)					//巡线直到药房，之后停止
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 7:
					if(Go_Forward(Task_Car,0.8,0.2,0.02) == 1)					//	确保停止的瞬间偏转角度不会改变太大
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;//这一步就到药房了，等待卸载
				case 8:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//等待卸载药品，之后回城
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,1.5,0.94,0.02))							//右转,其实是向后转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;//下一步是识别到左转的路口
				case 10:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,594),1,0,Recv_Buff,1.0) == 1)					//巡线直到左手十字路口
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//下一步左转
				case 11:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 12:
					if(1 == Turn_Left(Task_Car,1.5,0.46,0.05))							//左转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 13:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,593),1,0,Recv_Buff,1.0) == 2)					//巡线直到右手十字路口
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//下一步右转
				case 14:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//补距离
					{
						a_temp_step++;														
					}
					break;
				case 15:
					if(1 == Turn_Right(Task_Car,1.5,0.46,0.05))							//右转
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;//下一步直接开巡线环直到病房就ok了
				case 16://这个不清楚十字路口对巡线的干扰，实在不行遇到十字路口就直接打开位置环走过去再开巡线环
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,592),1,0,Recv_Buff,1.0) == 4)								//巡线前进
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//任务序号进入死区								
					}
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_FLASH:
				HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_3);		//点灯庆祝
				//Medicine_Car_Task++;							
				break;
		case APP_CAR_CONTROL_STOP:									
				break;		
		default:
			break;																		//结束之后所有任务标志位全处于结束停滞区，不复位就不会再次执行任何东西
	}
}





