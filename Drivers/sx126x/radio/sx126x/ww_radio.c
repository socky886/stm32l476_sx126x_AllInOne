#include "ww_radio.h"

bool IrqFired = false;
bool RxContinuous = false;
uint8_t MaxPayloadLength = 0xFF;
uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;
uint8_t RadioRxPayload[255];
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

typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

PacketStatus_t RadioPktStatus;
SX126x_t SX126x;

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
    //  RadioEvents = 0;

    SX126xInit( NULL );
    SX126xSetStandby( STDBY_RC );
    SX126xSetRegulatorMode( USE_DCDC );

    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    // Add registers to the retention list (4 is the maximum possible number)
    sx126x_dev_AddRegisterToRetentionList( REG_RX_GAIN );
    sx126x_dev_AddRegisterToRetentionList( REG_TX_MODULATION );

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
            // TimerStop(&RxTimeoutTimer);
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
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = sx126x_dev_GetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided
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
    // TxTimeout = timeout;

   

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
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = sx126x_dev_GetFskBandwidthRegValue( bandwidth << 1 ); // SX126x badwidth is double sided

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
static void sx126x_dev_SetModem( RadioModems_t modem )
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
            sx126x_dev_SetPublicNetwork(RadioPublicNetwork.Current);
        }
        break;
    }
}

static void sx126x_dev_AddRegisterToRetentionList( uint16_t registerAddress )
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

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t sx126x_dev_GetFskBandwidthRegValue( uint32_t bandwidth )
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


static void sx126x_dev_SetPublicNetwork( bool enable )
{
    printf("//--------------------------------------------------------------------\n");
    printf("RadioSetPublicNetwork\n{\n");

    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    sx126x_dev_SetModem( MODEM_LORA );
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

