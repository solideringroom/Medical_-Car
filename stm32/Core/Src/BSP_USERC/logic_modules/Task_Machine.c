
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   ��ҩС����״̬��
  * @brief   
  ******************************************************************************
  * @attention	
  * 		�ȴ���ʵ����ǰ������ת����ת�����˻ص�ԭ�㡣
				ÿһ��������������ɣ�ÿ�������������֮��᷵��һ����־�������ܸ��±�־λ��ִ����һ������
  * 		
  *			ȷ���������ٲ���С����
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
// ����λ����Ϣ����������ʾ�������ʵ�ʵ�ͼ���
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
	//����1����ҩ�͵����˲���
	switch (Medicine_Car_Task)
	{
		case APP_CAR_CONTROL_INIT:
			if(1 == Car_Init(Task_Car))
			{
				Medicine_Car_Task++;				//��һ����ʼ���ɹ�
			}
		
			break;
		case APP_CAR_CONTROL_WAIT_FOR_LOAD: //ҩƷװ�����
			if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
			{
				Medicine_Car_Task++;
			}
			break;
		case APP_CAR_CONTROL_ESTABLISH_COMMUNICATION:  //�����͸��Ӿ���
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
		case APP_CAR_CONTROL_WAIT_FOR_NUM:    //��һ����ʼ������ʶ��ȴ����Ӿ��Խ�
			if(1)
			{
				Medicine_Car_Task = APP_CAR_CONTROL_GOTO_F;//********************************�Ȳ���һ��Զ�˲���
				Car_FindLine_Mode_ON(Task_Car);	//�ȴ�ѭ��PID��׼��ѭ��
			}
			break;
		case APP_CAR_CONTROL_GOTO_A:
			//a_temp_step = 9;//������
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,0.5,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,0.8,0.46,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,1000),1,0,Recv_Buff,1.0) == 4)					//Ѳ��ֱ��ҩ����֮��ֹͣ
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 4:
					if(Go_Forward(Task_Car,0.5,0.2,0.02) == 1)					//	ȷ��ֹͣ��˲��ƫת�ǶȲ���ı�̫��
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;
				case 5:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//�ȴ�ж��ҩƷ��֮��س�
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 6:
					if(1 == Turn_Right(Task_Car,0.7,0.92,0.03))							//��ת,��ʵ�����ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 7:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//Ѳ��ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 8:
					
					if(1 == Go_Forward(Task_Car,0.5,0.75,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,0.8,0.5,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 10:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,999),1,0,Recv_Buff,1.0) == 4)								//Ѳ��ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//������Ž�������								
					}
					break;
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_GOTO_D:
			//a_temp_step = 9;//������
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//ǰ��
					{
						if(delay_timer(DELAY_TASK_5,1500)) //·�ڼ���Ҫ��ȴһ��ʱ��
						{
							time_flag = 0;
						}
						if(time_flag == 0)
						{
							flag_counter +=1;
							time_flag = 1;
						}
						if(flag_counter ==2) //�ڶ���ʮ��·��
						{
							Car_FindLine_Mode_OFF(Task_Car);
							Car_Position_Mode_ON(Task_Car);
							flag_counter = 0;
							a_temp_step++;
						}							
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,0.5,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,0.8,0.46,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,998),1,0,Recv_Buff,1.0) == 4)					//Ѳ��ֱ��ҩ����֮��ֹͣ
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 4:
					if(Go_Forward(Task_Car,0.5,0.2,0.02) == 1)					//	ȷ��ֹͣ��˲��ƫת�ǶȲ���ı�̫��
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;
				case 5:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//�ȴ�ж��ҩƷ��֮��س�
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 6:
					if(1 == Turn_Right(Task_Car,0.7,0.92,0.02))							//��ת,��ʵ�����ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 7:
					if(Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0) == 3)								//Ѳ��ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;																	
					}
					break;
				case 8:
					
					if(1 == Go_Forward(Task_Car,0.5,0.75,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,0.8,0.5,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 10://��������ʮ��·�ڶ�Ѳ�ߵĸ��ţ�ʵ�ڲ�������ʮ��·�ھ�ֱ�Ӵ�λ�û��߹�ȥ�ٿ�Ѳ�߻�
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,997),1,0,Recv_Buff,1.0) == 4)								//Ѳ��ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//������Ž�������								
					}
					break;
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_GOTO_F:
			//a_temp_step = 9;//������
			switch (a_temp_step)
			{
				case 0:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,697),1,0,Recv_Buff,1.0) == 3)								//ǰ��
					{
						if(delay_timer(DELAY_TASK_5,1000)) //·�ڼ���Ҫ��ȴһ��ʱ��
						{
							time_flag = 0;
						}
						if(time_flag == 0)
						{
							flag_counter +=1;
							time_flag = 1;
						}
						if(flag_counter == 3) //������ʮ��·��
						{
							Car_FindLine_Mode_OFF(Task_Car);
							Car_Position_Mode_ON(Task_Car);
							flag_counter = 0;
							a_temp_step++;
						}							
					}
					break;
				case 1:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Left(Task_Car,1.5,0.46,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 3:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,596),1,0,Recv_Buff,1.0) == 3)					//Ѳ��ֱ��ʮ��·��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//Ȼ��Ҫ��ת
				case 4:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 5://��ת
					if(Turn_Right(Task_Car,1.5,0.46,0.05) == 1)					
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 6://
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,595),1,0,Recv_Buff,1.0) == 4)					//Ѳ��ֱ��ҩ����֮��ֹͣ
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;
				case 7:
					if(Go_Forward(Task_Car,0.8,0.2,0.02) == 1)					//	ȷ��ֹͣ��˲��ƫת�ǶȲ���ı�̫��
					{
						Car_Position_Mode_OFF(Task_Car);
						a_temp_step++;												
					}
					break;//��һ���͵�ҩ���ˣ��ȴ�ж��
				case 8:
					if(1 == HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))							//�ȴ�ж��ҩƷ��֮��س�
					{
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 9:
					if(1 == Turn_Right(Task_Car,1.5,0.94,0.02))							//��ת,��ʵ�����ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;//��һ����ʶ����ת��·��
				case 10:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,594),1,0,Recv_Buff,1.0) == 1)					//Ѳ��ֱ������ʮ��·��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//��һ����ת
				case 11:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 12:
					if(1 == Turn_Left(Task_Car,1.5,0.46,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;
				case 13:
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,593),1,0,Recv_Buff,1.0) == 2)					//Ѳ��ֱ������ʮ��·��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Car_Position_Mode_ON(Task_Car);
						a_temp_step++;												
					}
					break;//��һ����ת
				case 14:
					if(1 == Go_Forward(Task_Car,2,0.8,0.05))							//������
					{
						a_temp_step++;														
					}
					break;
				case 15:
					if(1 == Turn_Right(Task_Car,1.5,0.46,0.05))							//��ת
					{
						Car_Position_Mode_OFF(Task_Car);
						Car_FindLine_Mode_ON(Task_Car);
						a_temp_step++;														
					}
					break;//��һ��ֱ�ӿ�Ѳ�߻�ֱ��������ok��
				case 16://��������ʮ��·�ڶ�Ѳ�ߵĸ��ţ�ʵ�ڲ�������ʮ��·�ھ�ֱ�Ӵ�λ�û��߹�ȥ�ٿ�Ѳ�߻�
					if(Car_Find_Line(Task_Car,slop_function(&speed_slop,0,Base_Speed,592),1,0,Recv_Buff,1.0) == 4)								//Ѳ��ǰ��
					{
						Car_FindLine_Mode_OFF(Task_Car);
						Medicine_Car_Task = APP_CAR_CONTROL_FLASH;
						a_temp_step++;									//������Ž�������								
					}
				default:
					break;	
			}
			break;
		case APP_CAR_CONTROL_FLASH:
				HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_3);		//�����ף
				//Medicine_Car_Task++;							
				break;
		case APP_CAR_CONTROL_STOP:									
				break;		
		default:
			break;																		//����֮�����������־λȫ���ڽ���ͣ����������λ�Ͳ����ٴ�ִ���κζ���
	}
}





