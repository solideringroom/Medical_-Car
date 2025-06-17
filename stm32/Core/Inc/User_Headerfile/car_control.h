#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#include "stm32f4xx_hal.h"
#include "pid_config.h"
#include "car_params_config.h"
#include "Encoder.h"
#include "TB6612.h"
#include "soft_timer.h"

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
uint8_t Go_Forward(CarType* Car,float speed_rev,float distance,float DEAD_ZONE);
uint8_t Go_Backward(CarType* Car,float speed_rev,float distance,float DEAD_ZONE);
uint8_t Turn_Left(CarType* Car,float speed_rev,float distance,float DEAD_ZONE);
uint8_t Turn_Right(CarType* Car,float speed_rev,float distance,float DEAD_ZONE);
void Car_Position_Mode_OFF(CarType *Car);
void Car_FindLine_Mode_ON(CarType *Car);
void Car_FindLine_Mode_OFF(CarType *Car);

void ERROR_EARSER_INIT();
extern float Left_Motor_Position_PID_Out,Left_Motor_Speed_PID_Out;
extern float Right_Motor_Position_PID_Out,Right_Motor_Speed_PID_Out;
uint8_t Car_Find_Line(CarType* Car,float Base_Speed,uint8_t Mode,float Mode_Gain,int* pack,float Over_All_Gain); //Mode指的是回正还是寻中线 Mode Gain指的是如果是回正模式的话PID误差成比例减小，那么增大其输出

#endif