#ifndef __SOFTTIMER_H
#define __SOFTTIMER_H
#include "stm32f4xx_hal.h"

//定义非阻塞延时定时器
typedef struct {
    uint32_t time_temp;
    uint8_t  delay_flag;
} Delay_Task;

// 定义软件定时器的枚举类型
typedef enum
{
    SOFT_TIMER_0,
    SOFT_TIMER_1,
    SOFT_TIMER_2,
    SOFT_TIMER_3,
    SOFT_TIMER_4,
    SOFT_TIMER_5,
    SOFT_TIMER_6,
    SOFT_TIMER_7,
    SOFT_TIMER_8,
    SOFT_TIMER_9,
    SOFT_TIMER_MAX // 定时器的最大数量
} soft_timer_type;

typedef enum
{
    DELAY_TASK_0,
    DELAY_TASK_1,
    DELAY_TASK_2,
    DELAY_TASK_3,
    DELAY_TASK_4,
    DELAY_TASK_5,
    DELAY_TASK_6,
    DELAY_TASK_7,
    DELAY_TASK_8,
    DELAY_TASK_9,
    MAX_DELAY_TASK // 定时器的最大数量
} delay_timer_type;
// 定义软件定时器的结构体
typedef struct
{
    volatile uint32_t counter;   // 计数器
    volatile uint32_t timeout;   // 超时时间
    volatile uint8_t is_timeout; // 超时标志
    volatile uint8_t is_activate;  // 是否激活
} software_timer;
typedef struct 
{
	uint32_t get_time;
	uint8_t time_flag;
	float slop_k;
	uint8_t if_init;
  float start;
	float end;
	uint32_t time;
}slope_function;

void soft_timer_repeat_init(soft_timer_type timer, uint32_t timeout);
uint8_t soft_timer_is_timeout(soft_timer_type timer);
void soft_timer_tick(void);
uint8_t delay_timer(delay_timer_type delay_task,uint32_t delay_time);
float slop_function(slope_function* slop,float start,float end,uint16_t time);


#endif