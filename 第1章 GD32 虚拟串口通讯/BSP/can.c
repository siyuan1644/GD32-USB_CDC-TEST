#include "can.h"
#include "BSP.h"
#include	<string.h>



//GD32 60M
const uint8_t CANBAUD[10][4]={
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_1TQ,15}, //0 1M  
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_3TQ,CAN_BT_BS2_1TQ,15},  //1 800k      80%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,15},  //2 500k      87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,30}, //3 250k			 87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_13TQ,CAN_BT_BS2_2TQ,30} //4 125k			 87.5% 
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,20},//5 100k 
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,83}, //6 62.5k  42M/((1+5+2)*83)
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,60}, //7 50k
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,210},//8 33.3k  42M/((1+2+3)*210)
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,84},//9 25k
};

//GD32 60M
const uint8_t CANBAUDFd[10][4]={
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_3TQ,CAN_BT_BS2_1TQ,12}, //0	 1M 80%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_3TQ,CAN_BT_BS2_1TQ,6}, //1	 2M 80% 
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_3TQ,CAN_BT_BS2_1TQ,3}, //2	 4M 80% 
 //{CAN_BT_SJW_1TQ,CAN_BT_BS1_8TQ,CAN_BT_BS2_3TQ,1}, //3	 5M 75% 
 {CAN_DBT_SWJ_1TQ,CAN_BT_BS1_9TQ,CAN_BT_BS2_2TQ,1}, //3	 5M 83.3% 
};
//	  can_fd_parameter.data_resync_jump_width=CAN_BT_SJW_1TQ;
//		can_fd_parameter.data_time_segment_1=CAN_BT_BS1_11TQ;
//		can_fd_parameter.data_time_segment_2=CAN_BT_BS2_3TQ;
//		can_fd_parameter.data_prescaler=1;//BAUD=60m/((1+6+1)*15)  87.5%

//		CAN_InitSt.resync_jump_width=CAN_BT_SJW_1TQ;
//		CAN_InitSt.time_segment_1=CAN_BT_BS1_6TQ;
//		CAN_InitSt.time_segment_2=CAN_BT_BS2_1TQ;





#define DEV_CAN0_ID          0xaabb
#define DEV_CAN0_MASK        0x0000
#define DEV_CAN1_ID          0xccdd
#define DEV_CAN1_MASK        0x0000
/* config CAN baud rate to 500K Hz (range from 1Hz to 1MHz)*/
#define DEV_CAN_BAUD_RATE    500000

//GD32  CAN =60M

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
   
		gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);//
	
		gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);//RX
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);//TX
	
//    /* configure CAN0 GPIO */
//    gpio_init(GPIOD,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
//    gpio_init(GPIOD,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
//    
//    /* configure CAN1 GPIO */
//    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);//RX
//    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);//TX
    
   // gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP,ENABLE);
   // gpio_pin_remap_config(GPIO_CAN1_REMAP,ENABLE);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;
     
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);
  //  can_deinit(CAN1);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;  
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
   // can_init(CAN1, &can_parameter);
    
    /* config CAN0 baud rate */
    can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);
    /* config CAN1 baud rate */
   // can_frequency_set(CAN1, DEV_CAN_BAUD_RATE);
    
    /* initialize filter */ 
    can1_filter_start_bank(14);
    can_filter_mask_mode_init(DEV_CAN0_ID, DEV_CAN0_MASK, CAN_STANDARD_FIFO0, 0);
    //can_filter_mask_mode_init(DEV_CAN1_ID, DEV_CAN1_MASK, CAN_EXTENDED_FIFO0, 15);
    
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* configure CAN1 NVIC */
  //  nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
  //  can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}



/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_configEx(uint8_t speed)
{
    can_parameter_struct CAN_InitSt;
	
     
    can_struct_para_init(CAN_INIT_STRUCT, &CAN_InitSt);
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN parameters */
    CAN_InitSt.time_triggered = DISABLE;
    CAN_InitSt.auto_bus_off_recovery = DISABLE;
    CAN_InitSt.auto_wake_up = DISABLE;
	
//		0：使能自动重发
//		1：禁用自动重发
    CAN_InitSt.auto_retrans = DISABLE;//报文自动传输 是否开启 
	
    CAN_InitSt.rec_fifo_overwrite = DISABLE;
    CAN_InitSt.trans_fifo_order = DISABLE;
    CAN_InitSt.working_mode = CAN_NORMAL_MODE;  
  
	//SPEED
		CAN_InitSt.resync_jump_width=CANBAUD[speed][0];
		CAN_InitSt.time_segment_1=CANBAUD[speed][1];
		CAN_InitSt.time_segment_2=CANBAUD[speed][2];
		CAN_InitSt.prescaler=CANBAUD[speed][3];//BAUD=60m/((1+6+1)*15)  87.5%
		
		/* initialize CAN */
    can_init(CAN0, &CAN_InitSt);

  
    
		//fiter 
    /* initialize filter */ 
   // can1_filter_start_bank(14);
  //  can_filter_mask_mode_init(0x7E8, 0x7E8, CAN_STANDARD_FIFO0, 0);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}






/*
*********************************************************************************************************
*                                          CAN1_Config16BitFilter()
*
* 功能   ： 设置CAN滤波器，过两个16位标准帧ID
*
* 参数   ： id1 ：要过的一个16位标准帧ID
*
*           id2 ：要过的另一个16位标准帧ID
*
* 返回值 ： 无
*
* 注释   ： 无
*********************************************************************************************************
*/
void CAN1_Config16BitFilter(u16 id1, u16 id2)
{
    can_filter_parameter_struct  CAN_FilterInitStructure;
		can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);

		CAN_FilterInitStructure.filter_number = 0;
    CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
    CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_16BIT;
	
    CAN_FilterInitStructure.filter_enable = ENABLE;
		 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
    /* configure SFID[10:0] */
    CAN_FilterInitStructure.filter_list_high = (uint16_t)id1 ;
    CAN_FilterInitStructure.filter_list_low = (uint16_t)id2;
    /* configure SFID[10:0] mask */
    CAN_FilterInitStructure.filter_mask_high = (uint16_t)id1;
    /* both data and remote frames can be received */
    CAN_FilterInitStructure.filter_mask_low = (uint16_t)id2 ;	
		can_filter_init(&CAN_FilterInitStructure);
}


//不过滤ID
void CAN_setAllfit(void)
{
		can_filter_parameter_struct  CAN_FilterInitStructure;
		can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);

		CAN_FilterInitStructure.filter_number = 0;
    CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_MASK;
    CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_32BIT;
	
    CAN_FilterInitStructure.filter_enable = ENABLE;
		 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
    /* configure SFID[10:0] */
    CAN_FilterInitStructure.filter_list_high = (uint16_t)0 ;
    CAN_FilterInitStructure.filter_list_low = (uint16_t)0;
    /* configure SFID[10:0] mask */
    CAN_FilterInitStructure.filter_mask_high = (uint16_t)0;
    /* both data and remote frames can be received */
    CAN_FilterInitStructure.filter_mask_low = (uint16_t)0 ;	
		can_filter_init(&CAN_FilterInitStructure);
}



// 列表模式 下 0~13 组 每组最多 4个STCAN ID 最多4*14=56个 ID
// FIFO 0  0~13
// FIFO 1  0~13 
//FIFO 共享 0~13 过滤器组
void CAN1_Config16BitFilterList(u8 iSumFiter,u16 *iBuffer)
{
    can_filter_parameter_struct  CAN_FilterInitStructure;
		u16 id1=0,id2=0,id3=0,id4=0;	
		u8 i=0,iCmt=0;
		u8 iSum=0;
		
		iSum=iSumFiter/4;
		if(iSumFiter%4)
		{
			iSum+=1;
		}
		if(iSum>28) iSum=28;	//fifo 1
	
		for(i=0;i<iSum;i++)//0~13  sum=14*2 
		{			
//			if(i>13) iFiterCount=i-14;//fifo 1
//			else iFiterCount=i;

			id1=iBuffer[iCmt++];
			id2=iBuffer[iCmt++];
			id3=iBuffer[iCmt++];
			id4=iBuffer[iCmt++];
			if(id1==0) id1=0xffff;
			if(id2==0) id2=0xffff;
			if(id3==0) id3=0xffff;
			if(id4==0) id4=0xffff;		
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_16BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint16_t)id1 ;
			CAN_FilterInitStructure.filter_list_low = (uint16_t)id2;
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint16_t)id3;
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint16_t)id4 ;	
			can_filter_init(&CAN_FilterInitStructure);

		}

 
    
}



//列表模式下  有0~13 共14组 能过滤 28个扩展帧ID
void CAN1_Config32BitFilterExList(u8 iSumFiter,u32 *iBuffer)
{
    u32 id1=0,id2=0;
		u8 i=0,iCmt=0,iSum=0;
    can_filter_parameter_struct  CAN_FilterInitStructure;
	//  id1=id1>>3;
		
		iSum=iSumFiter/2;
		if(iSumFiter%2)
		{
			iSum+=1;
		}
	
	
	
		for(i=0;i<iSum;i++)
		{
			id1=iBuffer[iCmt++];
			id2=iBuffer[iCmt++];
			
			id1=id1>>3;
			id2=id2>>3;
			
			if(id1==0) id1=0xffffffff;
			if(id2==0) id2=0xffffffff;
			
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_32BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint32_t)(id1>>13);
			CAN_FilterInitStructure.filter_list_low = (uint32_t)((id1<<3)|4);
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint32_t)(id1>>13);
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint32_t)((id2<<3)|4);
			can_filter_init(&CAN_FilterInitStructure);
			
		}
	

  
	

}




/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void canFd_config(u8 speed1,u8 speed2)
{
    can_parameter_struct can_parameter;
    can_fdframe_struct can_fd_parameter; 
    can_fd_tdc_struct can_fd_tdc_parameter;
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);

    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = ENABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;  
	
		can_parameter.resync_jump_width=CANBAUD[speed1][0];
		can_parameter.time_segment_1=CANBAUD[speed1][1];
		can_parameter.time_segment_2=CANBAUD[speed1][2];
		can_parameter.prescaler=CANBAUD[speed1][3];//BAUD=60m/((1+6+1)*15)  87.5%
	
    /* initialize CAN */
    can_init(CAN0, &can_parameter);

   
    /* config CAN0 baud rate */
  //  can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);

    
    can_struct_para_init(CAN_FD_FRAME_STRUCT, &can_fd_parameter);
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
    can_fd_tdc_parameter.tdc_filter = 0x04; 
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = CAN_FDMOD_ISO;
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;
//		    uint8_t data_resync_jump_width;                                     /*!< CAN resynchronization jump width */
//    uint8_t data_time_segment_1;                                        /*!< time segment 1 */
//    uint8_t data_time_segment_2;                                        /*!< time segment 2 */
//    uint16_t data_prescaler;                                            /*!< baudrate prescaler */
	//	CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_1TQ,15
	
			can_fd_parameter.data_resync_jump_width=CANBAUDFd[speed2][0];
		can_fd_parameter.data_time_segment_1=CANBAUDFd[speed2][1];
		can_fd_parameter.data_time_segment_2=CANBAUDFd[speed2][2];
		can_fd_parameter.data_prescaler=CANBAUDFd[speed2][3];//BAUD=60m/((1+6+1)*15)  87.5%
	

    can_fd_init(CAN0, &can_fd_parameter);
    
    //can_fd_frequency_set(CAN0, 4000000);

    

    can_filter_mask_mode_init(DEV_CAN0_ID, DEV_CAN0_MASK, CAN_EXTENDED_FIFO0, 0);
    
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);

    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);

}

//CAN FD
void SendCanFd(void)
{
	can_trasnmit_message_struct g_transmit_message;

	  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x00;
    g_transmit_message.tx_efid = 0x00;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_EXTENDED;
    g_transmit_message.tx_dlen = 12;
    g_transmit_message.fd_flag = 1;
    g_transmit_message.fd_brs = CAN_BRS_ENABLE;
    g_transmit_message.fd_esi = CAN_ESI_DOMINANT;

    /* initialize receive message */
   // can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &g_receive_message);
    g_transmit_message.tx_efid = 0xccdd;
            g_transmit_message.tx_data[0] = 0xaa;
            g_transmit_message.tx_data[1] = 0xbb;
            g_transmit_message.tx_data[2] = 0xcc;
            g_transmit_message.tx_data[3] = 0xdd;
            g_transmit_message.tx_data[4] = 0xee;
            g_transmit_message.tx_data[5] = 0xff;
            g_transmit_message.tx_data[6] = 0x00;
            g_transmit_message.tx_data[7] = 0x11;
            g_transmit_message.tx_data[8] = 0x22;
            g_transmit_message.tx_data[9] = 0x33;
            g_transmit_message.tx_data[10] = 0x44;
            g_transmit_message.tx_data[11] = 0x55;
						
						            /* transmit message */
   can_message_transmit(CAN0, &g_transmit_message);
}

void StCanInitTest(void)
{
	can_gpio_config();
	//can_config();
	can_configEx(can_500k);
	CAN1_Config16BitFilter(0xFD00,0xFD00);
	//CAN_setAllfit();
}

void StCanfDInitTest(void)
{
	can_gpio_config();
	//can_config();
	canFd_config(can_500k,Fdcan_5M);
	CAN1_Config16BitFilter(0xFD00,0xFD00);
	//CAN_setAllfit();
}
