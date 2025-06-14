#ifndef _MV_UART_H__
#define _MV_UART_H__
#include "main.h"
#include "stdio.h"
void Decode(uint8_t uart_recv_data,uint8_t* uart_buff,float* data_x,float* data_y);
uint8_t Transmit(uint8_t* data);
extern uint8_t recv_status;
extern uint8_t T_Data;	
#endif