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
 * \file       radio.h
 * \brief      Generic radio driver ( radio abstraction )
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Gregory Cristian on Apr 25 2013
 */
#ifndef __RADIO1_H__
#define __RADIO1_H__
#include <stdint.h>
#include <stdbool.h>
/*!
 * SX1272 and SX1276 General parameters definition
 */
#define LORA       0        // [0: OFF, 1: ON]

/*!
 * RF process function return codes
 */
typedef enum
{
    RF_IDLE,  
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_LEN_ERROR,
    RF_CHANNEL_EMPTY,
    RF_CHANNEL_ACTIVITY_DETECTED,
}tRFProcessReturnCodes;    //�������߽ӿڵ�״̬

/*!
 * Radio driver structure defining the different function pointers
 */
typedef struct sRadioDriver
{
    void ( *Init )( void );
    void ( *Reset )( void );
    void ( *RfsetPw )( uint8_t );     //�趨���߷��书��
    void ( *StartRx )( uint8_t );
    void ( *StartTx )( uint8_t );
    void ( *GetRxPacket )( void *buffer, uint16_t size );
    void ( *SetTxPacket )( const void *buffer, uint16_t size );
    uint8_t (*Process )( void );
}tRadioDriver;

/*!
 * \brief Initializes the RadioDriver structure with specific radio
 *        functions.
 *
 * \retval radioDriver Pointer to the radio driver variable
 */
tRadioDriver* RadioDriverInit( void );
void SI446xSetRFState( uint8_t state );    //�趨����оƬ�ĵ�ǰ״̬
uint8_t SI446xGetRFState(void);            //��ȡ����оƬ�ĵ�ǰ״̬
#endif // __RADIO_H__
