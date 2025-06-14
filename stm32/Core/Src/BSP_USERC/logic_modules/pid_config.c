
	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID�ĳ�ʼ�������㣬�����رգ��������
  * @brief   
  ******************************************************************************
  * @attention	
  * 
  * 
  *		
  * 
  * 
  * ʹ�ã��ȴ���PID�ṹ�壬Ȼ�󴫽�positional_pid_controller_init()˳�����ò�����ע��parainit��ǰ���Ǹ�init���ù�����
	*	��Ҫ����������ʼ��
	* 
  ******************************************************************************
  */
#include "pid_config.h"
#include <math.h>

#define ABS(x) ((x > 0) ? x : -x)
#define a 0
//ʹ��ʱֻ�趨��һ��positional_pid_ctrler�ṹ�����Ȼ���������ctrler_init�������渳ֵһЩ����
//�ṹ�����涨���������������ֱ�Ϊ�����޸ģ�PID�����رգ�PID���㡣����������ֱ�ӵ��ö�������ĺ���
//�ܽ᣺����һ���ṹ�����init��ʹ�øö������溯��


static void positional_pid_params_init(positional_pid_ctrler *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, 
																	float _separation_threshold,
                                  float _integral_limit_max,
                                  float _integral_limit_min,
																	float _output_max,float _output_min
																	)
{
    // ��ʼ�� PID ����
    positional_pid->kp = _kp;
    positional_pid->ki = _ki;
    positional_pid->kd = _kd;

    // ��ʼ��������������޺��������
    positional_pid->dead_zone = _dead_zone;
    positional_pid->output_max = _output_max;
    positional_pid->output_min = _output_min;

    // ��ʼ��Ŀ��ֵ�����ֵ
    positional_pid->target = 0;
    positional_pid->output = 0;
	
		// ��ʼ������������
		positional_pid->separation_threshold = _separation_threshold;
    positional_pid->integral_limit_max = _integral_limit_max;
    positional_pid->integral_limit_min = _integral_limit_min;
	
}




float positional_pid_compute(positional_pid_ctrler *positional_pid,
                             float _target, float _measure,Filter_Average* Filter_ERROR)
{
    if (positional_pid->control == PID_ENABLE)//���û����PID��PID���Ϊ��
    {
        // ����Ŀ��ֵ�Ͳ���ֵ
        positional_pid->target = _target;
        positional_pid->measure = _measure;

        // ��ȡ���
        positional_pid->error =
        positional_pid->target - positional_pid->measure;
				//��ͨ�˲���a=0û������
				//positional_pid->error = (1-a)*positional_pid->error + a*positional_pid->last_error;
        positional_pid->error = get_filtered_speed(Filter_ERROR,positional_pid->error);
			if (ABS(positional_pid->error) > positional_pid->dead_zone)//�����Ƿ񳬹����������������Ϊ��
        {
            // ���������
            positional_pid->p_out = positional_pid->kp * positional_pid->error;
            // ����΢����		����С,D���������,���Ż����ƹ���
            positional_pid->d_out =
                positional_pid->kd
                * (positional_pid->error - positional_pid->last_error);

            // ���ַ��룬���Ŀ��ֵ�Ͳ���ֵ��Ҳ���������󣬻��ֿ�����������ϵͳ���ȶ���
            // ֻ�����С�ڻ��ַ�����ֵ���Ž��л��ּ���
            if (ABS(positional_pid->error) < positional_pid->separation_threshold)
            {
                // ���������
                positional_pid->i_out += positional_pid->ki * positional_pid->error;
                // �����޷�
                if (positional_pid->i_out > positional_pid->integral_limit_max)
                {
                    positional_pid->i_out = positional_pid->integral_limit_max;
                }
                if (positional_pid->i_out < positional_pid->integral_limit_min)
                {
                    positional_pid->i_out = positional_pid->integral_limit_min;
                }
            }
            else
            {
                positional_pid->i_out = 0.0f;
            }

            positional_pid->last_error = positional_pid->error;//PID�����ּ�����֮��������

            // �����������P I D������
            positional_pid->output = positional_pid->p_out
                                     + positional_pid->i_out
                                     + positional_pid->d_out;
					}
			else//��������
				{
						positional_pid->output = 0.0f;
				}

			// ���������������޺��������֮��
			if (positional_pid->output > positional_pid->output_max)
				{
					positional_pid->output = positional_pid->output_max;
				}
			if (positional_pid->output < (positional_pid->output_min))
				{
					positional_pid->output = positional_pid->output_min;
				}

			// ������һ�β���ֵ�����ֵ�����ֵ
			positional_pid->last_error = positional_pid->error;

			return positional_pid->output;
		}

    else//û��PID����
    {
				positional_pid->output = 0;
        return 0.0f;
    }
}

void positional_pid_control(positional_pid_ctrler *positional_pid,pid_status _status)
{
    // ���� PID ��ʹ��״̬
    positional_pid->control = _status;
}

void PID_I_Reset(positional_pid_ctrler *positional_pid)
{
	positional_pid->i_out = 0;
}

void positional_pid_controller_init(positional_pid_ctrler *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, 
																	float _separation_threshold,
                                  float _integral_limit_max,
                                  float _integral_limit_min,
																	float _output_max,float _output_min)
{
    // ��ʼ�� PID ������
    positional_pid->positional_pid_params_init = positional_pid_params_init;
    positional_pid->positional_pid_control = positional_pid_control;
	  positional_pid->positional_pid_compute = positional_pid_compute;
    // ���ó�ʼ���������ò���
    positional_pid->positional_pid_params_init(positional_pid,
                                   _kp,  _ki,  _kd,
                                   _dead_zone, 
																	 _separation_threshold,
                                   _integral_limit_max,
                                   _integral_limit_min,
																	 _output_max, _output_min
																	);
    // Ĭ��ʹ�� PID ������
    positional_pid->control = PID_ENABLE;
}