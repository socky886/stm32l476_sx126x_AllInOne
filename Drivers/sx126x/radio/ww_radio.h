#ifndef __WW__RADIO__H__
#define __WW__RADIO__H__
#include "gfsk.h"
#include "sx126x_all.h"

/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/**
 * @brief wangpei 20210520-------------------------------------------------------------------------
 * 
 */


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

void sx126x_dev_SetPW( uint8_t rf_pW);     
void sx126x_dev_StartRx( uint8_t Channel);
void sx126x_dev_StartTx( uint8_t Channel );
void sx126x_dev_GetRxPacket( void *buffer, uint16_t size );
void sx126x_dev_SetTxPacket( const void *buffer, uint16_t size );
uint8_t sx126x_dev_Process( void );

void sx126x_dev_init(void);
void sx126x_dev_reset(void);
void sx126x_dev_Interupt(void);

void sx126x_dev_Rx( uint32_t timeout );
void sx126x_dev_Send( uint8_t *buffer, uint8_t size );

//static function
static void sx126x_dev_SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preambleLen,
                          bool fixLen, bool crcOn, bool FreqHopOn,
                          uint8_t HopPeriod, bool iqInverted, uint32_t timeout);
                          

static void sx126x_dev_SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous);
                        
static void sx126x_dev_SetMaxPayloadLength( RadioModems_t modem, uint8_t max );
static void sx126x_dev_SetModem( RadioModems_t modem );
static void sx126x_dev_AddRegisterToRetentionList( uint16_t registerAddress );
static uint8_t sx126x_dev_GetFskBandwidthRegValue( uint32_t bandwidth );
static void sx126x_dev_SetPublicNetwork( bool enable );

//---------------------------------------------------
typedef struct sFSKSettings
{
    uint8_t Channel;                       //Ƶ���� 0~200
    int8_t  Power;                         //������� 0~128 
}tFSKSettings;


typedef enum
{
    RFLR_STATE_IDLE,
    RFLR_STATE_RX_INIT,
    RFLR_STATE_RX_RUNNING,
    RFLR_STATE_RX_DONE,
    RFLR_STATE_RX_TIMEOUT,
    RFLR_STATE_TX_INIT,
    RFLR_STATE_TX_RUNNING,
    RFLR_STATE_TX_DONE,
    RFLR_STATE_TX_TIMEOUT,
    RFLR_STATE_CAD_INIT,
    RFLR_STATE_CAD_RUNNING,
    RFLR_STATE_CAD_DETECTED,     
    RFLR_STATE_CRC_ERROR,
}tRFLRStates;        

#endif