#include "timer.h"
#include "bsp.h"
///*定时器1用于计时器*/
//void TIM1_config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  /* Time base configuration */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//  TIM_DeInit(TIM1);
//  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;//初值
//  TIM_TimeBaseStructure.TIM_Prescaler = 3599*2;//预分频值
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分割
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;
//  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//  /* Prescaler configuration *///0.1ms RCC
//  //TIM_PrescalerConfig(TIM1, 7199, TIM_PSCReloadMode_Immediate);
//  TIM_Cmd(TIM1, ENABLE);
//}


///*******************************************************************************
//* Function Name  : TIM3_config
//* Description    : 通用定时器timer3初始化 TIM_Period=计数，TIM_Prescaler=分频系数
//* Input          : none
//* Output         : None
//* Return         : None  1ms 精度
//*******************************************************************************/

//void TIM2_config(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_TimeBaseStructure.TIM_Period = 100-1; //  0~65535    1000*10us=1ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(840-1); // 分频系数  10us 
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //时钟分割
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //
// 
//	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);  //时能中断
//	TIM_Cmd(TIM2, ENABLE);  //
//}



///*******************************************************************************
//* Function Name  : TIM3_config
//* Description    : 通用定时器timer3初始化 TIM_Period=计数，TIM_Prescaler=分频系数
//* Input          : none
//* Output         : None
//* Return         : None


//*******************************************************************************/

//void TIM3_config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	
//	TIM_TimeBaseStructure.TIM_Period = 2-1; //  0~65535    1us*2=2us
//	TIM_TimeBaseStructure.TIM_Prescaler =(84-1); // 分频系数 84M/84=1000khz=1us
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //时钟分割
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //
// 
//	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
//	TIM_Cmd(TIM3, ENABLE);  //
//							 
//}

//定时器5 16位，1us 计数
void TIM5_config(void)
{
	/* ----------------------------------------------------------------------------
    TIMER5 Configuration: 
    TIMER5CLK = SystemCoreClock/12000 = 10KHz, the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER5);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
	
		timer_initpara.period            = 0xFFFE;//溢出值
    timer_initpara.prescaler         = 120-1;//1us  
   // timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;//不分频
    timer_init(TIMER5, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    
    timer_enable(TIMER5);
		
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER5_IRQn, 1, 1);
		iTime5Cmt=0;//清空计数器
}




//定时器6 16位，1us 计数
//不产生溢出中断 用于时间测量
void TIM6_config(void)
{
	/* ----------------------------------------------------------------------------
    TIMER5 Configuration: 
    TIMER5CLK = SystemCoreClock/12000 = 10KHz, the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER6);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
	
		timer_initpara.period            = 0xFFFE;//溢出值
    timer_initpara.prescaler         = 120-1;//1us  
   // timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;//不分频
    timer_init(TIMER6, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);
    
    timer_enable(TIMER6);
//		
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER6_IRQn, 1, 1);
		iTime6Cmt=0;//清空计数器
}




/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none

VPW  PB6  TIMER3_CH0
PWM PB7  TIMER3_CH1

  */
void J1850_config(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz, the PWM frequency is 62.5Hz.

    TIMER1 channel0 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel1 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel2 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);
	
	
	
	  rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA0/PA1/PA2(TIMER1 CH0/CH1/CH2) as alternate function*/
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
	
	
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 120-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 64-1;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER3, TIMER_CH_0, &timer_ocinitpara);


    /* CH0 configuration in PWM mode0, duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, 32);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);





    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
}



/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none

VPW  PB6  TIMER3_CH0
PWM PB7  TIMER3_CH1

  */
void J1850_PWMconfig(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz, the PWM frequency is 62.5Hz.

    TIMER1 channel0 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel1 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel2 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    timer_deinit(TIMER3);
	
	
	
	  rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA0/PA1/PA2(TIMER1 CH0/CH1/CH2) as alternate function*/
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
	
	
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 120-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 64;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER3, TIMER_CH_1, &timer_ocinitpara);


    /* CH0 configuration in PWM mode0, duty cycle 25% */
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 32);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);





    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER3);
    /* auto-reload preload enable */
    timer_enable(TIMER3);
}

/*********************************
TIME5 延时 us 级别


**********************************/
void Time5Delayus(uint32_t us)
{
		uint32_t iCmt=0;
	
	
		uint32_t iSum=us/60000;//60ms
		
	for(uint32_t i=0;i<iSum;i++)
	{
		iTime5Cmt=0;
		timer_counter_value_config(TIMER5,0);//计数器清0
		while(1)
		{
			iCmt=timer_counter_read(TIMER5);//读取计数器 值		
			if((iCmt+iTime5Cmt)>=60000) break;		
			
		}
	}
	
	iSum=us%60000;//60ms
	if(iSum>0)
	{
		iTime5Cmt=0;
		timer_counter_value_config(TIMER5,0);//计数器清0
		while(1)
		{
			iCmt=timer_counter_read(TIMER5);//读取计数器 值		
			if((iCmt+iTime5Cmt)>=iSum) break;		
			
		}
	}

		
}
/*****************************
1us 延时精度


*****************************/
void Delay_us(uint32_t us)
{
	Time5Delayus(us);
}

void Delay_ms(uint32_t ms)
{
	Time5Delayus(ms*1000);
}

//获取时间值
uint32_t GetTimer6Cnt(void)
{
	uint32_t ti=0;
	ti=timer_counter_read(TIMER6);//读取计数器 值	
	ti+=iTime6Cmt;
	timer_counter_value_config(TIMER6,0);//计数器清0
	iTime6Cmt=0;//清空全局计数器
	return ti;
}
