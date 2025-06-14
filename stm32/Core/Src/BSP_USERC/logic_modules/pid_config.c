
	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID的初始化，计算，开启关闭，积分清空
  * @brief   
  ******************************************************************************
  * @attention	
  * 
  * 
  *		
  * 
  * 
  * 使用：先创建PID结构体，然后传进positional_pid_controller_init()顺便配置参数。注意parainit在前面那个init调用过函数
	*	不要调用它来初始化
	* 
  ******************************************************************************
  */
#include "pid_config.h"
#include <math.h>

#define ABS(x) ((x > 0) ? x : -x)
#define a 0
//使用时只需定义一个positional_pid_ctrler结构体对象，然后把它传入ctrler_init函数里面赋值一些参数
//结构体里面定义了三个函数，分别为参数修改，PID开启关闭，PID计算。后续都可以直接调用对象里面的函数
//总结：定义一个结构体对象，init，使用该对象里面函数


static void positional_pid_params_init(positional_pid_ctrler *positional_pid,
                                  float _kp, float _ki, float _kd,
                                  float _dead_zone, 
																	float _separation_threshold,
                                  float _integral_limit_max,
                                  float _integral_limit_min,
																	float _output_max,float _output_min
																	)
{
    // 初始化 PID 参数
    positional_pid->kp = _kp;
    positional_pid->ki = _ki;
    positional_pid->kd = _kd;

    // 初始化死区、输出上限和输出下限
    positional_pid->dead_zone = _dead_zone;
    positional_pid->output_max = _output_max;
    positional_pid->output_min = _output_min;

    // 初始化目标值和输出值
    positional_pid->target = 0;
    positional_pid->output = 0;
	
		// 初始化积分项限制
		positional_pid->separation_threshold = _separation_threshold;
    positional_pid->integral_limit_max = _integral_limit_max;
    positional_pid->integral_limit_min = _integral_limit_min;
	
}




float positional_pid_compute(positional_pid_ctrler *positional_pid,
                             float _target, float _measure,Filter_Average* Filter_ERROR)
{
    if (positional_pid->control == PID_ENABLE)//如果没开启PID，PID输出为零
    {
        // 设置目标值和测量值
        positional_pid->target = _target;
        positional_pid->measure = _measure;

        // 获取误差
        positional_pid->error =
        positional_pid->target - positional_pid->measure;
				//低通滤波（a=0没开启）
				//positional_pid->error = (1-a)*positional_pid->error + a*positional_pid->last_error;
        positional_pid->error = get_filtered_speed(Filter_ERROR,positional_pid->error);
			if (ABS(positional_pid->error) > positional_pid->dead_zone)//看看是否超过死区，在死区输出为零
        {
            // 计算比例项
            positional_pid->p_out = positional_pid->kp * positional_pid->error;
            // 计算微分项		误差变小,D项输出负数,来放缓控制过程
            positional_pid->d_out =
                positional_pid->kd
                * (positional_pid->error - positional_pid->last_error);

            // 积分分离，如果目标值和测量值（也就是误差）过大，积分快速增大会造成系统不稳定。
            // 只有误差小于积分分离阈值，才进行积分计算
            if (ABS(positional_pid->error) < positional_pid->separation_threshold)
            {
                // 计算积分项
                positional_pid->i_out += positional_pid->ki * positional_pid->error;
                // 积分限幅
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

            positional_pid->last_error = positional_pid->error;//PID三部分计算完之后更新误差

            // 计算总输出，P I D加起来
            positional_pid->output = positional_pid->p_out
                                     + positional_pid->i_out
                                     + positional_pid->d_out;
					}
			else//在死区里
				{
						positional_pid->output = 0.0f;
				}

			// 限制输出在输出上限和输出下限之间
			if (positional_pid->output > positional_pid->output_max)
				{
					positional_pid->output = positional_pid->output_max;
				}
			if (positional_pid->output < (positional_pid->output_min))
				{
					positional_pid->output = positional_pid->output_min;
				}

			// 更新上一次测量值、输出值和误差值
			positional_pid->last_error = positional_pid->error;

			return positional_pid->output;
		}

    else//没开PID控制
    {
				positional_pid->output = 0;
        return 0.0f;
    }
}

void positional_pid_control(positional_pid_ctrler *positional_pid,pid_status _status)
{
    // 控制 PID 的使能状态
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
    // 初始化 PID 控制器
    positional_pid->positional_pid_params_init = positional_pid_params_init;
    positional_pid->positional_pid_control = positional_pid_control;
	  positional_pid->positional_pid_compute = positional_pid_compute;
    // 调用初始化函数设置参数
    positional_pid->positional_pid_params_init(positional_pid,
                                   _kp,  _ki,  _kd,
                                   _dead_zone, 
																	 _separation_threshold,
                                   _integral_limit_max,
                                   _integral_limit_min,
																	 _output_max, _output_min
																	);
    // 默认使能 PID 控制器
    positional_pid->control = PID_ENABLE;
}