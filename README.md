# GD32 USB_CDC TEST
 GD32 usb speed test
1. 硬件地址
   https://shop100592026.taobao.com/
   
2.	例程说明
   约定简单通讯协议，测试虚拟串口最大传输速度

(1)  测试平台
  	1 WIN10 32/64位（免驱动），WIN7 32 /64 需安装驱动
       2 UsbTest.exe 测试工具，传输速度500K 左右
(2)   通讯协议说明
上位机发送一帧数据格式55 AA LEN(2BYTE) DATA ...
下位机发送一帧数据格式55 AA LEN(2BYTE) DATA ...

3.	软件设计
     UsbTest 上位机软件按5K一次循环读取选择的文件内容，
按格式 55 AA LEN(2BYTE) DATA ... 发送给下位机，下位机接收完数据后按
55 AA LEN(2BYTE) DATA ... 格式发回给上位机，上位机根据接收到的DATA写进temp.txt 
文件中，当文件发送完成后对比temp.txt 和选择的文件可知是否丢失数据。

(1)  上位机说明
1. 如下图所示，Open打开串口，GetFile 选择文件后开始传输数据
 

2 传输完成后用BCompare软件对比发送和接收的文件，如下图所示 ,18.5M的文件并未丢失数据
   

(2)下位机代码说明
     1 APP.c 文件代码说明
   	
iUsbLen=0;  //接收的数据长度
     i UsbFlag=0;//接收完成标记 
	 iUsbLenPre=0;
	
    /* main loop */
  while (1) 
	{
		if(iUsbFlag==0x80)//一帧数据 接收完毕
		{
			SendUsbDate(&cdc_acm,iUsbBuf,iUsbLen);//发送数据返回给下位机
			iUsbLenPre=0;
			iUsbLen=0;  //接收的数据长度
			iUsbFlag=0;//接收完成标记 
			iCmt++;
			if(iCmt%2) gpio_bit_reset(GPIOB,GPIO_PIN_12);
			else gpio_bit_set(GPIOB,GPIO_PIN_12);
		}
		continue;
    }

2 cdc_acm_core.c文件代码说明
USB虚拟串口接收函数
 static uint8_t cdc_acm_out (usb_dev *udev, uint8_t ep_num)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];
    cdc->packet_receive = 1U;
    cdc->receive_length = ((usb_core_driver *)udev)->dev.transc_out[ep_num].xfer_count;
	  iUsbFlag=0;//接收完成标记 
		if(cdc->data[0]==0x55&&cdc->data[1]==0xAA)//帧开头
		{
				iUsbLen=cdc->data[2]*256+cdc->data[3];	
		}
		for(uint8_t i=0;i<cdc->receive_length;i++)
		{
			if(iUsbLenPre>=10239) break;
			iUsbBuf[iUsbLenPre++]=cdc->data[i];
		}
		
		if(iUsbLenPre>=iUsbLen||iUsbLenPre>10239)	
		{
			iUsbFlag=0x80;//接收完成标记 
		}			
		cdc_acm_data_receive(udev);
    return USBD_OK;
}
USB虚拟串口发送函数
void SendUsbDate(usb_dev *udev,uint8_t*Buf,uint32_t len)
{
    usb_cdc_handler *cdc = (usb_cdc_handler *)udev->dev.class_data[CDC_COM_INTERFACE];
    if (0U != len) 
		{
        cdc->packet_sent = 0U;
        usbd_ep_send (udev, CDC_DATA_IN_EP, (uint8_t*)(Buf), len);
				cdc->receive_length = 0U;
    }
}

