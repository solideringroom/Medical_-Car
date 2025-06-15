
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ����ͨ��
  * @brief   
  ******************************************************************************
  * @attention	
  * ͨ��Э�飺"@%d.%d.\r\n"������ȡ��������Ȼ�������Э������Լ���д������λ�����͵�����һ�¾���
  * �ȶ�ȡ@��\r\n֮������ݣ������Ǵ�ŵ���������
  *	Ȼ���ٵ���sscanf�������е���Ϣ�������ŵ�uint16_t�������档�������Ϳ����Լ��ظ�
  * 
  * 
  * ʹ�ã���ʼ��ʱ����main�￪���ж�HAL_UART_Receive_IT(&huart2,&uart_recv_data,1)��Ȼ����д
	*	�жϻص�����void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)�����ڸú�����β�ٴο����ж�
	* �ڸú����м����decode����
  ******************************************************************************
  */

#include "MV_UART.h"
extern UART_HandleTypeDef huart2;
uint8_t recv_status;//�յ���У������
uint8_t T_Data;			//���͵�����
int Recv_Buff[4];
uint8_t uart_buff[100];
void Decode(uint8_t uart_recv_data,int* Recv_Buff)
{
	static uint8_t uart_status;
	static uint8_t uart_buff_index;
	switch(uart_status)
	{
		case 0:
			if('@' == uart_recv_data)			//��ʼ�ж�
				{
					uart_status = 1;
					uart_buff_index = 0;
				}
			else if('^'== uart_recv_data)
			{
				uart_status = 4;
				uart_buff_index = 0;
			}
					break;
		case 1:
			if('\r' == uart_recv_data)		//��ֹ�ж�1
			{
				uart_status = 2;
			}
			else
				{
					uart_buff[uart_buff_index] = uart_recv_data;
					uart_buff_index++;
				}
				break;
		case 2:
			if('\n' == uart_recv_data)		//��ֹ�ж�2
			{
				uart_status = 0;
				uart_buff_index = 0;
				sscanf((char*)uart_buff,"%d.%d.",&Recv_Buff[0],&Recv_Buff[1]);//��ʽתһ�£���������
				break;
			}
		case 4:
			if('\r' == uart_recv_data)		//��ֹ�ж�1
			{
				uart_status = 5;
			}
			else
				{
					uart_buff[uart_buff_index] = uart_recv_data;
					uart_buff_index++;
				}
				break;
		case 5:
			if('\n' == uart_recv_data)		//��ֹ�ж�2
			{
				uart_status = 0;
				uart_buff_index = 0;
				sscanf((char*)uart_buff,"%d.",(int *)&recv_status);//��ʽתһ�£���������
				break;
			}
	}
}
//��У����Ƶķ������� ������յ����ݺͷ��͵Ĳ�һ����ô��������
//��������(1/2/3)->��һ�˽��յ���Ȼ��ѽ��ܵ����ݷ����������һ�£��������� ��һ�£���������
uint8_t Transmit(uint8_t* data)
{
	HAL_StatusTypeDef STATUS;
	
	if(recv_status+48 != (uint8_t)*data)
	{	
		STATUS = HAL_UART_Transmit(&huart2,data,1,100);
		if(0!=STATUS)
		{
				return 2;//˵��û����ȥ�����ʹ���
		}
		return 0;
	}
	else
	{
		recv_status = 0;//��ԭ
		return 1;//���յ���
	}

}
//��ʼ��ʱ�ȿ���HAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart2)
//	{
//		Decode();
//		HAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//	}
//}