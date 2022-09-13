#include "bsp.h"
//#include "timer.h"
#include	<string.h>

__IO uint32_t   iTimeCmt = 0;                      //Send byte counter
__IO uint32_t   iTime5Cmt = 0;   //定时器5计数器
__IO uint32_t   iTime6Cmt = 0;   //定时器6计数器

//can_receive_message_struct g_receive_message;

/**
    \brief      configure the TIMER1 interrupt
    \param[in]  none
    \param[out] none
    \retval     none
  */
void nvic_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_config(void)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/12000 = 10KHz, the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 11999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    
    timer_enable(TIMER1);
}


void LenInit(void)
{
	
			    /* enable the LED2 GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* configure LED2 GPIO port */ 
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX,GPIO_PIN_12);
	
	
//gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX,VpwPin);//PB6
	
		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_MAX,VpwPin);//PB6
	
	   gpio_bit_set(GPIOB,VpwPin|PwmPin);
    /* reset LED2 GPIO pin */
    gpio_bit_reset(GPIOB,GPIO_PIN_12);
	
	
}





