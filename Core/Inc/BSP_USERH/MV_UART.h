#ifndef _MV_UART_H__
#define _MV_UART_H__
#include "main.h"
#include "stdio.h"
void Decode(uint8_t uart_recv_data,uint8_t* uart_buff,uint16_t* data_x,uint16_t* data_y);
#endif