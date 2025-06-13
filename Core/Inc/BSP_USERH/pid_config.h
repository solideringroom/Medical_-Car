#ifndef __PID_CONFIG_H
#define __PID_CONFIG_H
#include "stdint.h"
#include "Encoder.h"

typedef enum
{
    PID_DISABLE, /* PID失能 */
    PID_ENABLE, /* PID使能 */
} pid_status;


typedef struct positional_pid_ctrler
{

    char control;
		//PID系数
    float kp;
    float ki;
    float kd;
		//PID滤波器
		Filter_Average FILTER_ERROR;
		//PID目标值和测量值
    float target;
    float measure;
		//PID的两次误差，用于算D
    float error;
    float last_error;
		//PID三部分的各自输出
    float p_out;
    float i_out;
    float d_out;
		//PID的总输出
    float output;
		//PID输出限幅
    float output_max;
    float output_min;

    //积分分离阈值（误差较小的时候再调节零点误差）
    float separation_threshold;
    //积分限幅数值（防止爆炸）
    float integral_limit_max;
    float integral_limit_min;
		//防止在零点附近来回调，也改善了D项对微小扰动的敏感性
    float dead_zone;
		//三个函数指针，指向
    void (*positional_pid_params_init)(struct positional_pid_ctrler *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, 
																	float _separation_threshold,
                                  float _integral_limit_max,
                                  float _integral_limit_min,
																	float _output_max,float _output_min
																	);
    void (*positional_pid_control)(
        struct positional_pid_ctrler *positional_pid,
        pid_status status);
		float(*positional_pid_compute)(struct positional_pid_ctrler *positional_pid,
                             float _target, float _measure,Filter_Average* Filter_ERROR);
} positional_pid_ctrler;


//PID函数
void positional_pid_controller_init(positional_pid_ctrler *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, 
																	float _separation_threshold,
                                  float _integral_limit_max,
                                  float _integral_limit_min,
																	float _output_max,float _output_min);			
																	
void positional_pid_control(positional_pid_ctrler *positional_pid,pid_status _status);
																	
float positional_pid_compute(positional_pid_ctrler *positional_pid,
                             float _target, float _measure,Filter_Average* Filter_ERROR);

void PID_I_Reset(positional_pid_ctrler *positional_pid);

#endif