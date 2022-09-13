/*!
    \file    app.c
    \brief   USB main routine for CDC device

    \version 2020-12-31, V1.0.0, firmware for GD32C10x
*/

/*
 硬件接口
led   PB12
CAN  PB8 PB9

USB
PA2 PA3

LIN   
uart1  PA2 PA3    RX必须要接上拉电阻
控制引脚 PB11


J1850
输入 
PWM_IN   PA0   TIMER1_CH0
VPW_IN    PA1   TIMER1_CH1

输出
PWM_OUT  PB6  TIMER3_CH0
VPM_OUT   PB7  TIMER3_CH1

*/

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

#include	<stdio.h>
#include "can.h"
#include "bsp.h"
#include "timer.h"


usb_core_driver cdc_acm;


extern uint8_t iUsbBuf[1024*10];//接收款冲区
extern uint16_t iUsbLen,iUsbLenPre;  //接收的数据长度
extern uint8_t  iUsbFlag;//接收完成标记 

/*!
    \brief      main routine will construct a USB mass storage device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
		uint8_t iCmt=0;
//		uint8_t Send[100]={0};
//		uint8_t SendRe[100]={0};
		
		can_trasnmit_message_struct g_transmit_message;
		

		
		
    usb_rcu_config();

    usb_timer_init();

    usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);

    usb_intr_config();
    
#ifdef USE_IRC48M
    /* CTC peripheral clock enable */
    rcu_periph_clock_enable(RCU_CTC);

    /* CTC configure */
    ctc_config();

    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET) {
    }
#endif

		//
		//StCanInitTest();//CAN test
	//	StCanfDInitTest();//
		TIM5_config();//用于定时
		TIM6_config();//用于测量时间
		
		
		
		
		/* initialize transmit message */
    can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x00;
    g_transmit_message.tx_efid = 0x00;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_STANDARD;
    g_transmit_message.tx_dlen = 8;
    /* initialize receive message */
  //  can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);
		//strcat((char*)Send,"GD32C103 USB TEST\r\n");
		
		LenInit();
	//	J1850_config();
		//J1850_PWMconfig();
		iUsbLen=0;  //接收的数据长度
   iUsbFlag=0;//接收完成标记 
	 iUsbLenPre=0;
		//等待USB 接受准备
		while(1)
		{
				if (USBD_CONFIGURED == cdc_acm.dev.cur_status) 
				{
            if (0U == cdc_acm_check_ready(&cdc_acm)) 
						{
              cdc_acm_data_receive(&cdc_acm);
							break;
            }
        }
		}
		
		
		
    /* main loop */
  while (1) 
	{
		
		
		if(iUsbFlag==0x80)//一帧数据 接收完毕
		{
			
			SendUsbDate(&cdc_acm,iUsbBuf,iUsbLen);
			iUsbLenPre=0;
			iUsbLen=0;  //接收的数据长度
			iUsbFlag=0;//接收完成标记 
			iCmt++;
			if(iCmt%2) gpio_bit_reset(GPIOB,GPIO_PIN_12);
			else gpio_bit_set(GPIOB,GPIO_PIN_12);
		}
		continue;

    }
}
