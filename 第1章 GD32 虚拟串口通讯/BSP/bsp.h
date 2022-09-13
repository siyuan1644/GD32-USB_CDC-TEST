#ifndef __BSP_H
#define __BSP_H	 
#include "gd32C10x.h"
#include "string.h"
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

extern __IO uint32_t  iTimeCmt;
extern __IO uint32_t  iTime5Cmt;   //定时器5计数器
extern __IO uint32_t   iTime6Cmt;   //定时器6计数器

/* configure the TIMER peripheral */
void timer_config(void);
/* configure the TIMER1 interrupt */
void nvic_config(void);


#define VpwPin   GPIO_PIN_6
#define PwmPin   GPIO_PIN_7


void LenInit(void);


#endif
