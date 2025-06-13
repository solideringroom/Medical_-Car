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
uint8_t delay_timer(uint8_t delay_task,uint32_t delay_time)//非阻塞方式延时，替代haldelay
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


