
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

TASK_STATUS Medicine_Car_Task;
uint8_t a_temp_step ;
uint8_t flip_flag0;
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
		case APP_CAR_CONTROL_FIND_RED_LINE:    //����Ѳ����
			if(1)
			{
				Car_Find_Line(Task_Car,Base_Speed,1,0,Recv_Buff,1.0);//���￨������һֱ����
			}
			break;
		case APP_CAR_CONTROL_GOTO_A:
			switch (a_temp_step)
			{
				case 0:
					if(1 == Go_Forward(Task_Car,1,8))								//ǰ��
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;																	
					}
					break;
				case 1:
					if(1 == Turn_Left(Task_Car,0.8,0.5))							//��ת
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;														
					}
					break;
				case 2:
					if(1 == Turn_Right(Task_Car,0.8,0.5))							//��ת
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;												
					}
					break;
				case 3:
					if(1 == Go_Backward(Task_Car,1,8))							//����
					{
						ERROR_DISTANCE_PID.i_out = 0;
						a_temp_step++;														
					}
					break;
				case 4:
					a_temp_step++;																		//����������������ͣ����
					Medicine_Car_Task++;
					break;
				default:
					break;	
			}
			break;
		
		case APP_CAR_CONTROL_FLASH:
				HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_3);		//�����ף
				Medicine_Car_Task++;
				Car_Position_Mode_OFF(Task_Car);								//�ر�����PID���
				
				break;
		case APP_CAR_CONTROL_STOP:
				Car_Position_Mode_OFF(Task_Car);		
				break;		
		default:
			break;																		//����֮�����������־λȫ���ڽ���ͣ����������λ�Ͳ����ٴ�ִ���κζ���
	}
}





