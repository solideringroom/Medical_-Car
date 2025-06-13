#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#include "stm32f4xx_hal.h"
#include "pid_config.h"
#include "car_params_config.h"
#include "Encoder.h"
#include "TB6612.h"

extern positional_pid_ctrler ERROR_DISTANCE_PID;
typedef struct CarType
{
	positional_pid_ctrler PID_Left_Motor_Speed;
	positional_pid_ctrler PID_Left_Motor_Distance;
	positional_pid_ctrler PID_Right_Motor_Speed;
	positional_pid_ctrler PID_Right_Motor_Distance;
} CarType;

uint8_t Car_Init(CarType *Car);
void Car_Position_Mode_ON(CarType *Car);
uint8_t Go_Forward(CarType* Car,float speed_rev,float distance);
uint8_t Go_Backward(CarType* Car,float speed_rev,float distance);
uint8_t Turn_Left(CarType* Car,float speed_rev,float distance);
uint8_t Turn_Right(CarType* Car,float speed_rev,float distance);
void Car_Position_Mode_OFF(CarType *Car);

void ERROR_EARSER_INIT();
extern float Left_Motor_Position_PID_Out,Left_Motor_Speed_PID_Out;
extern float Right_Motor_Position_PID_Out,Right_Motor_Speed_PID_Out;

#endif