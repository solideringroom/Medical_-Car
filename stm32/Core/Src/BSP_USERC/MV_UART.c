#include "MV_UART.h"
extern UART_HandleTypeDef huart2;

void Decode(uint8_t uart_recv_data,uint8_t* uart_buff,uint16_t* data_x,uint16_t* data_y)
{
	static uint8_t uart_status;
	static uint8_t uart_buff_index;
	switch(uart_status)
	{
		case 0:
			if('@' == uart_recv_data)			//∆ º≈–∂œ
				{
					uart_status = 1;
					uart_buff_index = 0;
				}
					break;
		case 1:
			if('\r' == uart_recv_data)		//÷’÷π≈–∂œ1
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
			if('\n' == uart_recv_data)		//÷’÷π≈–∂œ2
			{
				uart_status = 0;
				sscanf((char*)uart_buff,"%d.%d.",data_x,data_y);
				break;
			}
	}
}
//≥ı ºªØ ±œ»ø™∆ÙHAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart2)
//	{
//		Decode();
//		HAL_UART_Receive_IT(&huart2,&uart_recv_data,1);
//	}
//}