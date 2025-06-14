
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    软件定时器模块
  * @brief   裸机开发用来模拟多线程运行
  ******************************************************************************
  * @attention	
  * 注意：main里面只存放进程的判断语句，不要在判断之外再写别的命令。不然软件定时器会非常不准。也不要使用HAL_delay
  * 这种阻塞式延时，一定要使用挂起-接管这种延时，也就是delay_timer()函数
  *	
  * 每次使用delay_timer()都要传进去新的定时任务。时间到了它的返回值会变成1然后到下一次执行的时候再变成0，用在判断条件中
	*
	* 使用：一定先把soft_timer_tick放到systick的中断里面，相当于软件版的时钟源。delay_timer()不需要init，但是模拟线程的soft_timer_repeat_init(soft_timer_type timer, uint32_t timeout)需要
  * 调用一下。init几个就相当于开了几个线程。然后在if条件里面调用soft_timer_is_timeout(soft_timer_type timer)即可
	*
	* 
  ******************************************************************************
  */
#include "soft_timer.h"
#include "stm32f4xx_hal.h"

static software_timer timers[SOFT_TIMER_MAX];

Delay_Task delay_tasks[MAX_DELAY_TASK];
/**
 * 用于if条件判断的条件里面，实现非阻塞式定时
 * @param 定时器id(每次使用都要用不同id定时器)
 * @param ms
 * 
 */
uint8_t delay_timer(delay_timer_type delay_task,uint32_t delay_time)//非阻塞方式延时，替代hal_delay
{
	if(delay_tasks[delay_task].delay_flag ==0)
	{
		delay_tasks[delay_task].time_temp = HAL_GetTick()+delay_time;
		delay_tasks[delay_task].delay_flag = 1;
	}
	if(HAL_GetTick()<delay_tasks[delay_task].time_temp) 
	{
		return 0;
	}
	else
	{
		delay_tasks[delay_task].delay_flag = 0;
		return 1;
	}
}

void soft_timer_repeat_init(soft_timer_type timer, uint32_t timeout)//相当于线程初始化
{
    if (timer < SOFT_TIMER_MAX)
    {
        timers[timer].counter = 0;//目前计数
        timers[timer].timeout = timeout;//超时时间
        timers[timer].is_timeout = 0;//超时判断标志位
        timers[timer].is_activate = 1;//是否激活
    }
}

uint8_t soft_timer_is_timeout(soft_timer_type timer)//超时标志位判断
{
    uint8_t ret = 0;
    if (timer < SOFT_TIMER_MAX)
    {
        ret = timers[timer].is_timeout;
        if (ret)
        {
            timers[timer].is_timeout = 0;
        }
    }
    return ret;
}				


 /**
  -  @brief  在32的SysTick_Handler中断服务程序中调用，每1ms调用一次(在stm32f4xx_it.c里面)
  -  @note   None
  -  @param  None
  -  @retval None
 */
void soft_timer_tick(void)
{
    for (soft_timer_type timer = SOFT_TIMER_0; timer < SOFT_TIMER_MAX; ++timer)
    {
			if(timers[timer].is_activate==1)
			{
				if (++timers[timer].counter >= timers[timer].timeout)
        {
            timers[timer].is_timeout = 1;
            timers[timer].counter = 0;         
        }
			}
    }
}


