#ifndef __TIMER_H
#define __TIMER_H
#include "gd32C10x.h"

void TIM1_config(void);
void TIM2_config(void);
void TIM3_config(void);


void TIM5_config(void);//用于产生延时 
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

void TIM6_config(void);//用于测量时间 


void J1850_config(void);
void J1850_PWMconfig(void);
#endif


