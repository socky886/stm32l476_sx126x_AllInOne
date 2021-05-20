/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       radio.c
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.0 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */


#include "Mysys_pic.h"
#include "radio.h"
#include "si446x.h"
#include "RF_Si446x_Hand.h"


tRadioDriver RadioDriver;

void SI4463_SetPW( uint8_t rf_pW);     //设定无线发射功率
void SI446xStartRx( uint8_t Channel);
void SI446xStartTx( uint8_t Channel );
void SI446xGetRxPacket( void *buffer, uint16_t size );
void SI446xSetTxPacket( const void *buffer, uint16_t size );
uint8_t SI446xProcess( void );


tRadioDriver* RadioDriverInit( void )
{   
    RadioDriver.Init = RF446x_Init;                       //初始化
    RadioDriver.Reset = SI4463_RESET;                         //复位
    RadioDriver.RfsetPw=SI4463_SetPW;                        //设定发射功率
    RadioDriver.StartRx = SI446xStartRx;                     //开始接收
    RadioDriver.StartTx = SI446xStartTx;                     //开始发送
    RadioDriver.GetRxPacket = SI446xGetRxPacket;             //获取数据包
    RadioDriver.SetTxPacket = SI446xSetTxPacket;             //开始发送
    RadioDriver.Process = SI446xProcess;                     //过程处理
    
    return &RadioDriver;
}

//设定无线芯片的发射功率 0~127
/***************************
  * 修改发射功率函数
 * level=0~127
 * level=10  Ptx=5dbm
 * level=20  Ptx=10dbm
 * level=30  Ptx=13dbm
 * level=40  Ptx=15dbm
 * level=50  Ptx=17dbm
 * level=60  Ptx=18dbm
 * level=70  Ptx=19dbm
 * level=80  Ptx=20dbm
 * level=81~127  Ptx=20dbm
 **************************/
void SI4463_SetPW( uint8_t rf_pW)
{
  SI446xFSKSetTX_Power(rf_pW);

   }

//无线接口函数
void SI446xStartRx( uint8_t Channel)
{ 
 if(( Channel<=200)&&(FSKSettings.Channel!=Channel)) //如果接收设定的频道号>200，或者等于原来的频道，就进入原来的频道接收
    FSKSettings.Channel=Channel;    //更改频道     
    SI446xFSKSetRFState( RFLR_STATE_RX_INIT );
}

//无线接口函数
void SI446xStartTx( uint8_t Channel )
{ 
 if(( Channel<=200)&&(FSKSettings.Channel!=Channel)) //如果接收设定的频道号>200，或者等于原来的频道，就进入原来的频道接收
    FSKSettings.Channel=Channel;    //更改频道     
 SI446xFSKSetRFState( RFLR_STATE_TX_INIT );  
}


void SI446xSetRFState( uint8_t state )
{
    SI446xFSKSetRFState( state );
 }


uint8_t SI446xGetRFState(void)
{
    return SI446xFSKGetRFState();

}

void SI446xGetRxPacket( void *buffer, uint16_t size )
{
    SI4463GetRxPacket( buffer, size );
}


void SI446xSetTxPacket( const void *buffer, uint16_t size )
{ 
    SI4463SetTxPacket( buffer, size );
 
}

uint8_t SI446xProcess( void )
{
    return SI446xFSKProcess( );
}

