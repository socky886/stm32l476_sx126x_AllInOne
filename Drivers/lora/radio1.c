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

void SI4463_SetPW( uint8_t rf_pW);     //�趨���߷��书��
void SI446xStartRx( uint8_t Channel);
void SI446xStartTx( uint8_t Channel );
void SI446xGetRxPacket( void *buffer, uint16_t size );
void SI446xSetTxPacket( const void *buffer, uint16_t size );
uint8_t SI446xProcess( void );


tRadioDriver* RadioDriverInit( void )
{   
    RadioDriver.Init = RF446x_Init;                       //��ʼ��
    RadioDriver.Reset = SI4463_RESET;                         //��λ
    RadioDriver.RfsetPw=SI4463_SetPW;                        //�趨���书��
    RadioDriver.StartRx = SI446xStartRx;                     //��ʼ����
    RadioDriver.StartTx = SI446xStartTx;                     //��ʼ����
    RadioDriver.GetRxPacket = SI446xGetRxPacket;             //��ȡ���ݰ�
    RadioDriver.SetTxPacket = SI446xSetTxPacket;             //��ʼ����
    RadioDriver.Process = SI446xProcess;                     //���̴���
    
    return &RadioDriver;
}

//�趨����оƬ�ķ��书�� 0~127
/***************************
  * �޸ķ��书�ʺ���
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

//���߽ӿں���
void SI446xStartRx( uint8_t Channel)
{ 
 if(( Channel<=200)&&(FSKSettings.Channel!=Channel)) //��������趨��Ƶ����>200�����ߵ���ԭ����Ƶ�����ͽ���ԭ����Ƶ������
    FSKSettings.Channel=Channel;    //����Ƶ��     
    SI446xFSKSetRFState( RFLR_STATE_RX_INIT );
}

//���߽ӿں���
void SI446xStartTx( uint8_t Channel )
{ 
 if(( Channel<=200)&&(FSKSettings.Channel!=Channel)) //��������趨��Ƶ����>200�����ߵ���ԭ����Ƶ�����ͽ���ԭ����Ƶ������
    FSKSettings.Channel=Channel;    //����Ƶ��     
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

