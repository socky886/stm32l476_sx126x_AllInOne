/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "utilities.h"
#include "timer.h"
#include "delay.h"
#include "radio.h"
#include "sx126x.h"
#include "sx126x-board.h"
#include "board.h"
#include "lora.h"
/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit( RadioEvents_t *events );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom( void );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint32_t bandwidthAfc, uint16_t preambleLen,
                          uint16_t symbTimeout, bool fixLen,
                          uint8_t payloadLen,
                          bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                          bool iqInverted, bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preambleLen,
                          bool fixLen, bool crcOn, bool FreqHopOn,
                          uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency( uint32_t frequency );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi( RadioModems_t modem );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite( uint32_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead( uint32_t addr );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork( bool enable );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime( void );

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess( void );

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted( uint32_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

/*!
 * \brief Add a register to the retention list
 *
 * \param [in] registerAddress The address of the register to be kept in retention
 */
void RadioAddRegisterToRetentionList( uint16_t registerAddress );

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    RadioInit,//ok
    RadioGetStatus,//ok
    RadioSetModem,//ok
    RadioSetChannel,//ok
    RadioIsChannelFree,//ok
    RadioRandom,//ok
    RadioSetRxConfig,//ok
    RadioSetTxConfig,//ok
    RadioCheckRfFrequency,//ok
    RadioTimeOnAir,//ok
    RadioSend,//ok
    RadioSleep,//ok
    RadioStandby,//ok
    RadioRx,//ok
    RadioStartCad,//ok
    RadioSetTxContinuousWave,//ok
    RadioRssi,//ok
    RadioWrite,//ok
    RadioRead,//ok
    RadioWriteBuffer,//ok
    RadioReadBuffer,//ok
    RadioSetMaxPayloadLength,//ok
    RadioSetPublicNetwork,//ok
    RadioGetWakeupTime,//ok
    RadioIrqProcess,//ok
    // Available on SX126x only
    RadioRxBoosted, //ok
    RadioSetRxDutyCycle //ok
};

/*
 * Local types definition
 */


 /*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;


PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];

bool IrqFired = false;

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void* context );

/*!
 * \brief Tx timeout timer callback
 */
void RadioOnTxTimeoutIrq( void* context );

/*!
 * \brief Rx timeout timer callback
 */
void RadioOnRxTimeoutIrq( void* context );

/*
 * Private global variables
 */


/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
SX126x_t SX126x;

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    if( bandwidth == 0 )
    {
        return( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i+1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void RadioInit( RadioEvents_t *events )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioInit\n{\n");

    RadioEvents = events;

    SX126xInit( RadioOnDioIrq );
    SX126xSetStandby( STDBY_RC );
    SX126xSetRegulatorMode( USE_DCDC );

    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    // Add registers to the retention list (4 is the maximum possible number)
    RadioAddRegisterToRetentionList( REG_RX_GAIN );
    RadioAddRegisterToRetentionList( REG_TX_MODULATION );

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;

    printf("\n}\n");
}

RadioState_t RadioGetStatus( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioGetStatus\n{\n");



    switch( SX126xGetOperatingMode( ) )
    {
        case MODE_TX:
            return RF_TX_RUNNING;
        case MODE_RX:
            return RF_RX_RUNNING;
        case MODE_CAD:
            return RF_CAD;
        default:
            return RF_IDLE;
    }
}

void RadioSetModem( RadioModems_t modem )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetModem\n{\n");

    switch( modem )
    {
    default:
    case MODEM_FSK:
        SX126xSetPacketType( PACKET_TYPE_GFSK );
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        SX126xSetPacketType( PACKET_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }

    printf("\n}\n");
}

void RadioSetChannel( uint32_t freq )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetChannel\n{\n");


    SX126xSetRfFrequency( freq );

    printf("\n}\n");
}

bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioIsChannelFree\n{\n");

    bool     status           = true;
    int16_t  rssi             = 0;
    uint32_t carrierSenseTime = 0;

    RadioSetModem( MODEM_FSK );

    RadioSetChannel( freq );

    // Set Rx bandwidth. Other parameters are not used.
    RadioSetRxConfig( MODEM_FSK, rxBandwidth, 600, 0, rxBandwidth, 3, 0, false,
                      0, false, 0, 0, false, true );
    RadioRx( 0 );

    DelayMs( 1 );

    carrierSenseTime = TimerGetCurrentTime( );

    // Perform carrier sense for maxCarrierSenseTime
    while( TimerGetElapsedTime( carrierSenseTime ) < maxCarrierSenseTime )
    {
        rssi = RadioRssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
    }
    RadioSleep( );
    return status;
}

uint32_t RadioRandom( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioRandom\n{\n");

    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    RadioSetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    SX126xSetDioIrqParams( IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    rnd = SX126xGetRandom( );

    return rnd;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetRxConfig\n{\n");



    RxContinuous = rxContinuous;
    if( rxContinuous == true )
    {
        symbTimeout = 0;
    }
    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;

            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;
            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            // SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 2 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            // SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            // SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
             SX126xSetSyncWord( ( uint8_t[] ){ 0x2D, 0xD4, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );
            // SX126xSetWhiteningSeed( 0x0100 );

            RxTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;
            break;

        case MODEM_LORA:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate = ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;

            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            SX126xSetLoRaSymbNumTimeout( symbTimeout );

            // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
            if( SX126x.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED )
            {
                SX126xWriteRegister( REG_IQ_POLARITY, SX126xReadRegister( REG_IQ_POLARITY ) & ~( 1 << 2 ) );
            }
            else
            {
                SX126xWriteRegister( REG_IQ_POLARITY, SX126xReadRegister( REG_IQ_POLARITY ) | ( 1 << 2 ) );
            }
            // WORKAROUND END

            // Timeout Max, Timeout handled directly in SetRx function
            RxTimeout = 0xFFFF;

            break;
    }

    printf("\n}\n");
}

void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetTxConfig\n{\n");


    switch( modem )
    {
        case MODEM_FSK:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;

            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided
            SX126x.ModulationParams.Params.Gfsk.Fdev = fdev;

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            // SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3 ; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 2 << 3 ; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            // SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;//RADIO_DC_FREE_OFF
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            // SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetSyncWord( ( uint8_t[] ){ 0x2D, 0xD4, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );
            break;

        case MODEM_LORA:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t ) datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth =  Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate= ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;
            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            break;
    }

    // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    if( ( modem == MODEM_LORA ) && ( SX126x.ModulationParams.Params.LoRa.Bandwidth == LORA_BW_500 ) )
    {
        SX126xWriteRegister( REG_TX_MODULATION, SX126xReadRegister( REG_TX_MODULATION ) & ~( 1 << 2 ) );
    }
    else
    {
        SX126xWriteRegister( REG_TX_MODULATION, SX126xReadRegister( REG_TX_MODULATION ) | ( 1 << 2 ) );
    }
    // WORKAROUND END

    SX126xSetRfTxPower( power );
    TxTimeout = timeout;

    printf("\n}\n");
}

bool RadioCheckRfFrequency( uint32_t frequency )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioCheckRfFrequency\n{\n");

    return true;
}

static uint32_t RadioGetLoRaBandwidthInHz( RadioLoRaBandwidths_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case LORA_BW_007:
        bandwidthInHz = 7812UL;
        break;
    case LORA_BW_010:
        bandwidthInHz = 10417UL;
        break;
    case LORA_BW_015:
        bandwidthInHz = 15625UL;
        break;
    case LORA_BW_020:
        bandwidthInHz = 20833UL;
        break;
    case LORA_BW_031:
        bandwidthInHz = 31250UL;
        break;
    case LORA_BW_041:
        bandwidthInHz = 41667UL;
        break;
    case LORA_BW_062:
        bandwidthInHz = 62500UL;
        break;
    case LORA_BW_125:
        bandwidthInHz = 125000UL;
        break;
    case LORA_BW_250:
        bandwidthInHz = 250000UL;
        break;
    case LORA_BW_500:
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t RadioGetGfskTimeOnAirNumerator( uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    const RadioAddressComp_t addrComp = RADIO_ADDRESSCOMP_FILT_OFF;
    const uint8_t syncWordLength = 3;

    return ( preambleLen << 3 ) +
           ( ( fixLen == false ) ? 8 : 0 ) +
             ( syncWordLength << 3 ) +
             ( ( payloadLen +
               ( addrComp == RADIO_ADDRESSCOMP_FILT_OFF ? 0 : 1 ) +
               ( ( crcOn == true ) ? 2 : 0 ) 
               ) << 3 
             );
}

static uint32_t RadioGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    int32_t ceilDenominator;
    int32_t ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    int32_t intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioTimeOnAir\n{\n");

    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * RadioGetGfskTimeOnAirNumerator( datarate, coderate,
                                                                  preambleLen, fixLen,
                                                                  payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * RadioGetLoRaTimeOnAirNumerator( bandwidth, datarate,
                                                                  coderate, preambleLen,
                                                                  fixLen, payloadLen, crcOn );
            denominator = RadioGetLoRaBandwidthInHz( Bandwidths[bandwidth] );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void RadioSend( uint8_t *buffer, uint8_t size )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSend\n{\n");

    SX126xSetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( SX126xGetPacketType( ) == PACKET_TYPE_LORA )
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = size;
    }
    else
    {
        SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
    }
    SX126xSetPacketParams( &SX126x.PacketParams );

    SX126xSendPayload( buffer, size, 0 );
    TimerSetValue( &TxTimeoutTimer, TxTimeout );
    TimerStart( &TxTimeoutTimer );
}

void RadioSleep( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSleep\n{\n");

    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;
    SX126xSetSleep( params );

    DelayMs( 2 );
}

void RadioStandby( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioStandby\n{\n");

    SX126xSetStandby( STDBY_RC );
}

void RadioRx( uint32_t timeout )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioRx\n{\n");

    SX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
        SX126xSetRx( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        SX126xSetRx( RxTimeout << 6 );
    }
}

void RadioRxBoosted( uint32_t timeout )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioRxBoosted\n{\n");

    SX126xSetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( RxContinuous == true )
    {
        SX126xSetRxBoosted( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        SX126xSetRxBoosted( RxTimeout << 6 );
    }
}

void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetRxDutyCycle\n{\n");

    SX126xSetRxDutyCycle( rxTime, sleepTime );
}

void RadioAddRegisterToRetentionList( uint16_t registerAddress )
{
    uint8_t buffer[9];

    // Read the address and registers already added to the list
    SX126xReadRegisters( REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

    const uint8_t nbOfRegisters = buffer[0];
    uint8_t* registerList   = &buffer[1];

    // Check if the register given as parameter is already added to the list
    for( uint8_t i = 0; i < nbOfRegisters; i++ )
    {
        if( registerAddress == ( ( uint16_t ) registerList[2 * i] << 8 ) + registerList[2 * i + 1] )
        {
            return;
        }
    }

    if( nbOfRegisters < MAX_NB_REG_IN_RETENTION )
    {
        buffer[0] += 1;
        registerList[2 * nbOfRegisters]     = ( uint8_t )( registerAddress >> 8 );
        registerList[2 * nbOfRegisters + 1] = ( uint8_t )( registerAddress >> 0 );

        // Update radio with modified list
        SX126xWriteRegisters( REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
    }
}

void RadioStartCad( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioStartCad\n{\n");

    SX126xSetDioIrqParams( IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    SX126xSetCad( );
}

void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetTxContinuousWave\n{\n");

    uint32_t timeout = ( uint32_t )time * 1000;

    SX126xSetRfFrequency( freq );
    SX126xSetRfTxPower( power );
    SX126xSetTxContinuousWave( );

    TimerSetValue( &TxTimeoutTimer, timeout );
    TimerStart( &TxTimeoutTimer );
}

int16_t RadioRssi( RadioModems_t modem )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioRssi\n{\n");

    return SX126xGetRssiInst( );
}

void RadioWrite( uint32_t addr, uint8_t data )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioWrite\n{\n");
    SX126xWriteRegister( addr, data );
}

uint8_t RadioRead( uint32_t addr )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioRead\n{\n");
    return SX126xReadRegister( addr );
}

void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioWriteBuffer\n{\n");
    SX126xWriteRegisters( addr, buffer, size );
}

void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioReadBuffer\n{\n");
    SX126xReadRegisters( addr, buffer, size );
}

void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetMaxPayloadLength\n{\n");

    if( modem == MODEM_LORA )
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        SX126xSetPacketParams( &SX126x.PacketParams );
    }
    else
    {
        if( SX126x.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH )
        {
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
            SX126xSetPacketParams( &SX126x.PacketParams );
        }
    }
}

void RadioSetPublicNetwork( bool enable )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetPublicNetwork\n{\n");

    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem( MODEM_LORA );
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
        SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
    }
    else
    {
        // Change LoRa modem SyncWord
        SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
        SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
    }
}

uint32_t RadioGetWakeupTime( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioGetWakeupTime\n{\n");

    return SX126xGetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

void RadioOnTxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
    {
        RadioEvents->TxTimeout( );
    }
}

void RadioOnRxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
    {
        RadioEvents->RxTimeout( );
    }
}

void RadioOnDioIrq( void* context )
{
    IrqFired = true;
}

void RadioIrqProcess( void )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioIrqProcess\n{\n");

    CRITICAL_SECTION_BEGIN( );
    // Clear IRQ flag
    const bool isIrqFired = IrqFired;
    IrqFired = false;
    CRITICAL_SECTION_END( );

    if( isIrqFired == true )
    {
        uint16_t irqRegs = SX126xGetIrqStatus( );
        SX126xClearIrqStatus( irqRegs );

        // Check if DIO1 pin is High. If it is the case revert IrqFired to true
        CRITICAL_SECTION_BEGIN( );
        if( SX126xGetDio1PinState( ) == 1 )
        {
            IrqFired = true;
        }
        CRITICAL_SECTION_END( );

        if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
            TimerStop( &TxTimeoutTimer );
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            SX126xSetOperatingMode( MODE_STDBY_RC );
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
        }

        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            TimerStop( &RxTimeoutTimer );

            if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
            {
                if( RxContinuous == false )
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    SX126xSetOperatingMode( MODE_STDBY_RC );
                }
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxError ) )
                {
                    RadioEvents->RxError( );
                }
            }
            else
            {
                uint8_t size;

                if( RxContinuous == false )
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    SX126xSetOperatingMode( MODE_STDBY_RC );

                    // WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
                    SX126xWriteRegister( REG_RTC_CTRL, 0x00 );
                    SX126xWriteRegister( REG_EVT_CLR, SX126xReadRegister( REG_EVT_CLR ) | ( 1 << 1 ) );
                    // WORKAROUND END
                }
                SX126xGetPayload( RadioRxPayload, &size , 255 );
                SX126xGetPacketStatus( &RadioPktStatus );
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt );
                }
            }
        }

        if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
        {
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            SX126xSetOperatingMode( MODE_STDBY_RC );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) );
            }
        }

        if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
            if( SX126xGetOperatingMode( ) == MODE_TX )
            {
                TimerStop( &TxTimeoutTimer );
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode( MODE_STDBY_RC );
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
                {
                    RadioEvents->TxTimeout( );
                }
            }
            else if( SX126xGetOperatingMode( ) == MODE_RX )
            {
                TimerStop( &RxTimeoutTimer );
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode( MODE_STDBY_RC );
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
            }
        }

        if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
        {
            TimerStop( &RxTimeoutTimer );
            if( RxContinuous == false )
            {
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode( MODE_STDBY_RC );
            }
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
                RadioEvents->RxTimeout( );
            }
        }
    }
}

/**
 * @brief weijunfeng 20210520-----------------------------------------
 * 
 */

static tRFLRStates RFLRState = RFLR_STATE_IDLE; 
// Default settings
tFSKSettings FSKSettings =
{
    199,    //58,              // Channel (RFFrequency)
    127,                // Power  ???????
};

/**
 * @brief wangpei 20210520-------------------------------------------------------------------------
 * 
 */

tRadioDriver RadioDriver;
tRadioDriver* RadioDriverInit( void )
{   
    RadioDriver.Init = sx126x_dev_init;
    RadioDriver.Reset = sx126x_dev_reset;                        
    RadioDriver.RfsetPw=sx126x_dev_SetPW;                        
    RadioDriver.StartRx = sx126x_dev_StartRx;                    
    RadioDriver.StartTx = sx126x_dev_StartTx;                     
    RadioDriver.GetRxPacket = sx126x_dev_GetRxPacket;            
    RadioDriver.SetTxPacket = sx126x_dev_SetTxPacket;             
    RadioDriver.Process = sx126x_dev_Process;                     
    
    return &RadioDriver;
}

void sx126x_dev_SetPW( uint8_t rf_pW)
{
    SX126xSetRfTxPower(rf_pW);
}  

/**
 * @brief set channel
 *        set working state
 * 
 * @param Channel 
 */
void sx126x_dev_StartRx( uint8_t Channel)
{
    // set channel
    // int cha_space=250000;
    // uint32_t fre=433000000;
    // fre=fre+cha_space*Channel;
    SX126xSetRfFrequency( 433000000 );
    RFLRState=RFLR_STATE_RX_INIT;

}
/**
 * @brief set channel
 *        set working state
 * 
 * @param Channel 
 */
void sx126x_dev_StartTx( uint8_t Channel )
{
    // set channel
    // int cha_space=250000;
    // uint32_t fre=433000000;
    // fre=fre+cha_space*Channel;
    SX126xSetRfFrequency( 433000000 );
    RFLRState=RFLR_STATE_TX_INIT;

}
void sx126x_dev_GetRxPacket( void *buffer, uint16_t size )
{
    // memcpy( buffer, Rf_Rec_Str.Rf_Buffer, ( uint16_t )size );
}
void sx126x_dev_SetTxPacket( const void *buffer, uint16_t size )
{
    // memcpy( Rf_Tec_Str.Rf_Buffer, buffer, ( uint16_t )size ); 
    // RFLRState=RFLR_STATE_TX_INIT;
}
uint8_t sx126x_dev_Process( void )
{
     uint8_t result = RF_BUSY;  //????
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        result = RF_IDLE;           //???????
        break;
    case RFLR_STATE_RX_INIT:     // invoke api to receive
        sx126x_dev_Rx(0);
        RFLRState = RFLR_STATE_RX_RUNNING ;                    //?????????
        
        break;
    case RFLR_STATE_RX_RUNNING:     //????????
                              //??��?????????????????��
        result = RF_IDLE;           //???????
        break;                               //???result ???????BUSY
    case RFLR_STATE_RX_DONE:        //???????
         
        result = RF_RX_DONE;        //???????????
        break;
    case RFLR_STATE_RX_TIMEOUT:          //??????
        // NOP();
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:// invoke api to transmit
        
        sx126x_dev_Send(0,4);
         
        RFLRState = RFLR_STATE_TX_RUNNING;
        break;                       //???result ???????BUSY
    case RFLR_STATE_TX_RUNNING:      //?????????????????��????
        // if((EnableA_T(7))&&(EndA_T(7)))
        result = RF_TX_TIMEOUT;      //??????
        break;                       //???result ???????BUSY
    case RFLR_STATE_TX_DONE:         //??????????��?
      
        

        RFLRState = RFLR_STATE_RX_INIT;   //????????
        result = RF_TX_DONE;    
        break;
    case RFLR_STATE_CAD_INIT:           //????????
        // NOP();
        break;
    case RFLR_STATE_CAD_RUNNING:
        // NOP();
        break;
    case RFLR_STATE_CAD_DETECTED:
        // NOP();
        break;
    case RFLR_STATE_CRC_ERROR:
        // NOP();
        RFLRState = RFLR_STATE_RX_INIT;
        break;
    default:
        break;
    } 
    
    // if(Rf_chip_err_Status)     //????????CTS?????????????��??????��???????
    // result = RF_TX_TIMEOUT;
    // else if(Si4463_Working_Condition())
    // result = RF_TX_TIMEOUT;   //SI4463��???????
    return result;
}

void sx126x_dev_init(void)
{
    //sx126x io init --------------

    // sx126x base init-------------
    //  RadioEvents = events;
     RadioEvents = 0;

    SX126xInit( RadioOnDioIrq );
    SX126xSetStandby( STDBY_RC );
    SX126xSetRegulatorMode( USE_DCDC );

    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    // Add registers to the retention list (4 is the maximum possible number)
    RadioAddRegisterToRetentionList( REG_RX_GAIN );
    RadioAddRegisterToRetentionList( REG_TX_MODULATION );

    // Initialize driver timeout timers
    // TimerInit( &TxTimeoutTimer, RadioOnTxTimeoutIrq );
    // TimerInit( &RxTimeoutTimer, RadioOnRxTimeoutIrq );

    IrqFired = false;

    // Set Channel---------------------
    // SX126xSetRfFrequency( RF_FREQUENCY );
    SX126xSetRfFrequency( 433000000 );

    // set tx config
    sx126x_dev_SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      false, 0, 0, 0, 3000);
    // set rx config
    sx126x_dev_SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 4, true,
                      0, 0, false, true);
    // set max payload length
    sx126x_dev_SetMaxPayloadLength(MODEM_FSK, BUFFER_SIZE);

}
void sx126x_dev_reset(void)
{
    SX126xReset();
}

void sx126x_dev_Interupt(void)
{

    CRITICAL_SECTION_BEGIN();
    // Clear IRQ flag
    const bool isIrqFired = IrqFired;
    IrqFired = false;
    CRITICAL_SECTION_END();

    if (isIrqFired == true)
    {
        uint16_t irqRegs = SX126xGetIrqStatus();
        SX126xClearIrqStatus(irqRegs);

        // Check if DIO1 pin is High. If it is the case revert IrqFired to true
        CRITICAL_SECTION_BEGIN();
        if (SX126xGetDio1PinState() == 1)
        {
            IrqFired = true;
        }
        CRITICAL_SECTION_END();

        if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE)
        {
            // TimerStop(&TxTimeoutTimer);
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            SX126xSetOperatingMode(MODE_STDBY_RC);

            RFLRState=RFLR_STATE_RX_DONE;
        }

        if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE)
        {
            // TimerStop(&RxTimeoutTimer);

            if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
            {
                if (RxContinuous == false)
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    SX126xSetOperatingMode(MODE_STDBY_RC);
                }

                RFLRState =RFLR_STATE_CRC_ERROR;   
            }
            else
            {
                uint8_t size;

                if (RxContinuous == false)
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    SX126xSetOperatingMode(MODE_STDBY_RC);

                    // WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
                    SX126xWriteRegister(REG_RTC_CTRL, 0x00);
                    SX126xWriteRegister(REG_EVT_CLR, SX126xReadRegister(REG_EVT_CLR) | (1 << 1));
                    // WORKAROUND END
                }
                SX126xGetPayload(RadioRxPayload, &size, 255);
                SX126xGetPacketStatus(&RadioPktStatus);
                // if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                // {
                //     RadioEvents->RxDone( RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt );
                // }

                 RFLRState=RFLR_STATE_TX_DONE;
            }
        }

        if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE)
        {
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            SX126xSetOperatingMode(MODE_STDBY_RC);
        }

        if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
        {
            if (SX126xGetOperatingMode() == MODE_TX)
            {
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode(MODE_STDBY_RC);
                
            }
            else if (SX126xGetOperatingMode() == MODE_RX)
            {
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode(MODE_STDBY_RC);
            }
        }

        if ((irqRegs & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED)
        {
            //__NOP( );
        }

        if ((irqRegs & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID)
        {
            //__NOP( );
        }

        if ((irqRegs & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
        {
            //__NOP( );
        }

        if ((irqRegs & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR) // rx timeout
        {
            TimerStop(&RxTimeoutTimer);
            if (RxContinuous == false)
            {
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                SX126xSetOperatingMode(MODE_STDBY_RC);
            }
        }
    }
}
void sx126x_dev_Rx( uint32_t timeout )
{

    SX126xSetDioIrqParams(IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);

    if (timeout != 0)
    {
        // TimerSetValue(&RxTimeoutTimer, timeout);
        // TimerStart(&RxTimeoutTimer);
    }

    if (RxContinuous == true)
    {
        SX126xSetRx(0xFFFFFF); // Rx Continuous
    }
    else
    {
        SX126xSetRx(RxTimeout << 6);
    }
}

void sx126x_dev_Send( uint8_t *buffer, uint8_t size )
{

    SX126xSetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);

    if (SX126xGetPacketType() == PACKET_TYPE_LORA)
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = size;
    }
    else
    {
        SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
    }
    SX126xSetPacketParams(&SX126x.PacketParams);

    SX126xSendPayload(buffer, size, 0);
    // TimerSetValue( &TxTimeoutTimer, TxTimeout );
    // TimerStart( &TxTimeoutTimer );
}




// static function definition
static void sx126x_dev_SetTxConfig(RadioModems_t modem, int8_t power, uint32_t fdev,
                                   uint32_t bandwidth, uint32_t datarate,
                                   uint8_t coderate, uint16_t preambleLen,
                                   bool fixLen, bool crcOn, bool FreqHopOn,
                                   uint8_t HopPeriod, bool iqInverted, uint32_t timeout)
{
     switch( modem )
    {
        case MODEM_FSK:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;

            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided
            SX126x.ModulationParams.Params.Gfsk.Fdev = fdev;

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            // SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3 ; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 2 << 3 ; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            // SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;//RADIO_DC_FREE_OFF
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;

            SX126xSetStandby( STDBY_RC );
            sx126x_dev_SetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            // SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetSyncWord( ( uint8_t[] ){ 0x2D, 0xD4, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );
            break;

        case MODEM_LORA:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t ) datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth =  Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate= ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;
            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            SX126xSetStandby( STDBY_RC );
            sx126x_dev_SetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            break;
    }

    // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    if( ( modem == MODEM_LORA ) && ( SX126x.ModulationParams.Params.LoRa.Bandwidth == LORA_BW_500 ) )
    {
        SX126xWriteRegister( REG_TX_MODULATION, SX126xReadRegister( REG_TX_MODULATION ) & ~( 1 << 2 ) );
    }
    else
    {
        SX126xWriteRegister( REG_TX_MODULATION, SX126xReadRegister( REG_TX_MODULATION ) | ( 1 << 2 ) );
    }
    // WORKAROUND END

    SX126xSetRfTxPower( power );
    TxTimeout = timeout;

   

}

static void sx126x_dev_SetRxConfig(RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous)
{
     RxContinuous = rxContinuous;
    if( rxContinuous == true )
    {
        symbTimeout = 0;
    }
    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;

            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;
            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            // SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 2 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            // SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;

            SX126xSetStandby( STDBY_RC );
            sx126x_dev_SetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            // SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
             SX126xSetSyncWord( ( uint8_t[] ){ 0x2D, 0xD4, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );
            // SX126xSetWhiteningSeed( 0x0100 );

            RxTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;
            break;

        case MODEM_LORA:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate = ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;

            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            SX126xSetStandby( STDBY_RC );
            sx126x_dev_SetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            SX126xSetLoRaSymbNumTimeout( symbTimeout );

            // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
            if( SX126x.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED )
            {
                SX126xWriteRegister( REG_IQ_POLARITY, SX126xReadRegister( REG_IQ_POLARITY ) & ~( 1 << 2 ) );
            }
            else
            {
                SX126xWriteRegister( REG_IQ_POLARITY, SX126xReadRegister( REG_IQ_POLARITY ) | ( 1 << 2 ) );
            }
            // WORKAROUND END

            // Timeout Max, Timeout handled directly in SetRx function
            RxTimeout = 0xFFFF;

            break;
    }

}

void sx126x_dev_SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{

    if (modem == MODEM_LORA)
    {
        SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        SX126xSetPacketParams(&SX126x.PacketParams);
    }
    else
    {
        if (SX126x.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH)
        {
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
            SX126xSetPacketParams(&SX126x.PacketParams);
        }
    }
}

/**
 * @brief sx126x set modem
 * 
 * @param modem 
 */
void sx126x_dev_SetModem( RadioModems_t modem )
{

    switch (modem)
    {
    default:
    case MODEM_FSK:
        SX126xSetPacketType(PACKET_TYPE_GFSK);
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        SX126xSetPacketType(PACKET_TYPE_LORA);
        // Public/Private network register is reset when switching modems
        if (RadioPublicNetwork.Current != RadioPublicNetwork.Previous)
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork(RadioPublicNetwork.Current);
        }
        break;
    }
}




