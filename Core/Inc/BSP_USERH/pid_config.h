#ifndef __PID_CONFIG_H
#define __PID_CONFIG_H
#include "stdint.h"
#include "Encoder.h"

typedef enum
{
    PID_DISABLE, /* PIDʧ�� */
    PID_ENABLE, /* PIDʹ�� */
} pid_status;


typedef struct positional_pid_ctrler
{

    char control;
		//PIDϵ��
    float kp;
    float ki;
    float kd;
		//PID�˲���
		Filter_Average FILTER_ERROR;
		//PIDĿ��ֵ�Ͳ���ֵ
    float target;
    float measure;
		//PID��������������D
    float error;
    float last_error;
		//PID�����ֵĸ������
    float p_out;
    float i_out;
    float d_out;
		//PID�������
    float output;
		//PID����޷�
    float output_max;
    float output_min;

    //���ַ�����ֵ������С��ʱ���ٵ��������
    float separation_threshold;
    //�����޷���ֵ����ֹ��ը��
    float integral_limit_max;
    float integral_limit_min;
		//��ֹ����㸽�����ص���Ҳ������D���΢С�Ŷ���������
    float dead_zone;
		//��������ָ�룬ָ��
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


//PID����
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