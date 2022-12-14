#ifndef __CAN_H
#define __CAN_H
#include "BSP.h"

#define can_500k 2  //500k
#define can_250k 3  //250k
#define can_125k 4  //125k


#define Fdcan_1M    0   //1m
#define Fdcan_2M    1   //2m
#define Fdcan_4M    2   //4m
#define Fdcan_5M    3   //5m



//void Init_CAN(u8 speed);
//void Init_CANEx(u8 speedCmt);
//void CAN1_Config16BitFilter(u16 id1, u16 id2);
//void CAN1_Config32BitFilter(u32 id1,u32 id2);
//void CAN_setAllfit(void);
//void CAN_setstdfit(void);

//void CAN_Excanfit(void);

//void Send_Frame_CAN(u32 id,u8 *cmdaddr);   //
//void Send_Frame_EXCAN(u32 id,u8 *cmdaddr); 
//void Send_10Frame_CAN(u32 id,u8 *cmdaddr,u8 tmp); 
//void Send_MulFrame_CAN(u32 id,u8 *cmdaddr,u8 iRet,u8 count); 
//void CAN_setstdfit_EX(u16 ID1,u16 ID2);
//void CAN_Excanfit_EX(u32 ID1,u32 ID2);
//void CAN_setstdfit_EX_ALL(u16 sum,char data[]);
//void CAN_Excanfit_EX_ALL(u8 sum,char data[]);

//void Send_Frame_CANEx1(u32 id,u8 *cmdaddr);   //
//u8 Send_Frame_CANEx(u32 id,u8 Dlc,u8 Sid,u8 Pid); 
//u8 Send_Frame_StCanEx(u32 id,u8 *iReqCmd,u8 iRet,u8 *iAnsCmd);   //

//void CAN1_Config16BitFilter1(u8 iSumFiter,u16 *iBuffer);//过滤ID  掩码模式
//void CAN1_Config16BitFilterList(u8 iSumFiter,u16 *iBuffer);//列表模式

//void CAN1_Config32BitFilterEx(u8 iSumFiter,u32 *iBuffer);//
//void CAN1_Config32BitFilterExList(u8 iSumFiter,u32 *iBuffer);
//void CAN1_Config32BitFilterExMast(u8 iSumFiter,u32 *iBuffer);

//void CAN_setMastfit(u8 iSumFiter,u16 *iBuffer);//过滤ID 组
//void CAN1_Config16BitFilterList_test(void);

//u8 Send_Frame_CAN15765(u8 *cmdaddr);
//u8 Send_Frame_ExCAN15765(u8 *cmdaddr);   //
//u8 Send_Frame_VWCAN15765(u8 *cmdaddr);
//u8 Send_Frame_VWCAN15765Cmt(u8 *cmdaddr);


//void ReCan15765StCanData(void);//接收CAN 数据
//void ReVwCanTF80CanData(void);//接收 VW CAN 数据


//u8 Send_Frame_CAN15765_One(u8 *cmdaddr,u8 iReFlag);   // 15765 函数 带接收
//u8 Send_Frame_CAN15765_Mul(u8 *cmdaddr,u8 iReFlag);
//u8 Send_Frame_EXCAN15765_Mul(u8 *cmdaddr,u8 iReFlag);   //

//u8 Send_Frame_CAN15765_OneEx(u8 *cmdaddr,u8 iReFlag); 
//u8 Send_Frame_VWCAN_Mul(u8 *cmdaddr,u8 iReFlag);//VW CAN 


void StCanInitTest(void);
void StCanfDInitTest(void);

void CAN1_Config16BitFilter(u16 id1, u16 id2);
void CAN_setAllfit(void);

void SendCanFd(void);
#endif


