
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    串口通信
  * @brief   
  ******************************************************************************
  * @attention	
  * 通信协议："@%d.%d.\r\n"这样会取出两个自然数。这个协议可以自己重写，和上位机发送到数据一致就行
  * 先读取@和\r\n之间的内容，把他们存放到数组里面
  *	然后再调用sscanf把数组中的信息解析，放到uint16_t变量里面。这里类型可以自己重改
  * 
  * 
  * 使用：初始化时先在main里开启中断HAL_UART_Receive_IT(&huart2,&uart_recv_data,1)，然后重写
	*	中断回调函数void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)并且在该函数结尾再次开启中断
	* 在该函数中间调用decode函数
  ******************************************************************************
  */

#include "MV_UART.h"
extern UART_HandleTypeDef huart2;
uint8_t recv_status;//收到的校验数字
uint8_t T_Data;			//发送的数字
int Recv_Buff[4];
uint8_t uart_buff[100];
void Decode(uint8_t uart_recv_data,int* Recv_Buff)
{
	static uint8_t uart_status;
	static uint8_t uart_buff_index;
	switch(uart_status)
	{
		case 0:
			if('@' == uart_recv_data)			//起始判断
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
			if('\r' == uart_recv_data)		//终止判断1
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
			if('\n' == uart_recv_data)		//终止判断2
			{
				uart_status = 0;
				uart_buff_index = 0;
				sscanf((char*)uart_buff,"%d.%d.",&Recv_Buff[0],&Recv_Buff[1]);//格式转一下，消除警告
				break;
			}
		case 4:
			if('\r' == uart_recv_data)		//终止判断1
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
			if('\n' == uart_recv_data)		//终止判断2
			{
				uart_status = 0;
				uart_buff_index = 0;
				sscanf((char*)uart_buff,"%d.",(int *)&recv_status);//格式转一下，消除警告
				break;
			}
	}
}
//带校验机制的发送数据 如果回收的数据和发送的不一致那么继续发送
//发送数据(1/2/3)->那一端接收到，然后把接受的数据发回来。如果一致：不发送了 不一致：继续发送
uint8_t Transmit(uint8_t* data)
{
	HAL_StatusTypeDef STATUS;
	
	if(recv_status+48 != (uint8_t)*data)
	{	
		STATUS = HAL_UART_Transmit(&huart2,data,1,100);
		if(0!=STATUS)
		{
				return 2;//说明没发出去，发送错误
		}
		return 0;
	}
	else
	{
		recv_status = 0;//还原
		return 1;//接收到了
	}

}
//初始化时先开启HAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart2)
//	{
//		Decode();
//		HAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//	}
//}