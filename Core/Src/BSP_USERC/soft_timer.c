#include "soft_timer.h"
#include "stm32f4xx_hal.h"

static software_timer timers[SOFT_TIMER_MAX];

Delay_Task delay_tasks[MAX_DELAY_TASK];
/**
 * ����if�����жϵ��������棬ʵ�ַ�����ʽ��ʱ
 * @param ��ʱ��id(ÿ��ʹ�ö�Ҫ�ò�ͬid��ʱ��)
 * @param ms
 * 
 */
uint8_t delay_timer(uint8_t delay_task,uint32_t delay_time)//��������ʽ��ʱ�����haldelay
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

void soft_timer_repeat_init(soft_timer_type timer, uint32_t timeout)//�൱���̳߳�ʼ��
{
    if (timer < SOFT_TIMER_MAX)
    {
        timers[timer].counter = 0;//Ŀǰ����
        timers[timer].timeout = timeout;//��ʱʱ��
        timers[timer].is_timeout = 0;//��ʱ�жϱ�־λ
        timers[timer].is_activate = 1;//�Ƿ񼤻�
    }
}

uint8_t soft_timer_is_timeout(soft_timer_type timer)//��ʱ��־λ�ж�
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
  -  @brief  ��32��SysTick_Handler�жϷ�������е��ã�ÿ1ms����һ��(��stm32f4xx_it.c����)
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


