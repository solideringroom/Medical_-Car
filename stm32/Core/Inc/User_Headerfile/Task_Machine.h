#ifndef _TASK_MACHINE__
#define _TASK_MACHINE__
#include "car_control.h"

typedef enum
{
    APP_CAR_CONTROL_INIT = 0,
	  APP_CAR_CONTROL_WAIT_FOR_LOAD,
		APP_CAR_CONTROL_ESTABLISH_COMMUNICATION,
    APP_CAR_CONTROL_WAIT_FOR_K210_NUMBER,
    APP_CAR_CONTROL_GOTO_A,
		APP_CAR_CONTROL_FLASH,
    APP_CAR_CONTROL_STOP,
} TASK_STATUS;

extern TASK_STATUS Medicine_Car_Task;
extern uint8_t a_temp_step ;

void TASK_RUN(CarType* Task_Car);

#endif