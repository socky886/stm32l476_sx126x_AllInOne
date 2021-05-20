#include "lora.h"
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include "sx126x.h"
#include "board-config.h"
#include "sx126x-board.h"



const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );



void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}

/**
 * @brief packet init
 * 
 */
void packet_init(void)
{
    GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    SpiInit( &SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX126xIoInit( );
    // Radio initialization
    // RadioEvents.TxDone = OnTxDone;
    // RadioEvents.RxDone = OnRxDone;
    // RadioEvents.TxTimeout = OnTxTimeout;
    // RadioEvents.RxTimeout = OnRxTimeout;
    // RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_MODEM_LORA)

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

#elif defined(USE_MODEM_FSK)

    // Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
    //                   FSK_DATARATE, 0,
    //                   FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
    //                   true, 0, 0, 0, 3000);

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      false, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 4, true,
                      0, 0, false, true);

    Radio.SetMaxPayloadLength(MODEM_FSK, BUFFER_SIZE);

#else
#error "Please define a frequency band in the compiler options."
#endif
}

/**
 * @brief 
 * 
 */
void jf_packet_init(void)
{
    GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    SpiInit(&SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC);
    SX126xIoInit();

    // init
    sx126x_dev_init();
}
/**
 * @brief transmit packet
 * 
 */
void packet_tx(void)
{
    int i;
    uint16_t irqRegs;
    // Indicates on a LED that the received frame is a PONG
    // GpioToggle(&Led1);

    // Send the next PING frame
    // Buffer[0] = 'P';
    // Buffer[1] = 'I';
    // Buffer[2] = 'N';
    // Buffer[3] = 'G';

    Buffer[0] = 0x12;
    Buffer[1] = 0x23;
    Buffer[2] = 0x45;
    Buffer[3] = 0x67;
    Buffer[4] = 0xff;
    Buffer[5] = 0x00;
    Buffer[6] = 0xf0;

    // We fill the buffer with numbers for the payload
    BufferSize=4;
    // for (i = 4; i < BufferSize; i++)
    // {
    //     Buffer[i] = i - 4;
    // }
    Radio.Standby();
    
    DelayMs(2);
    irqRegs=SX126xGetIrqStatus();
    SX126xClearIrqStatus(irqRegs);
    // SX126xClearIrqStatus(0xffff);
    vRadio_Hal_Start();
    Radio.Send(Buffer, BufferSize);
    // SX126xClearIrqStatus(0xffff);
    i=0;
    while (1)
    {
        irqRegs=SX126xGetIrqStatus();
        printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_TX_DONE) ==IRQ_TX_DONE)
        {
            printf("transmit finished\n");
            vRadio_Hal_end();
            break;
        }
        else
        {
            break;
            printf("--------\n");
            i++;
            HAL_Delay(10);
            if(i>=200)
            {
                printf("transmit timeout\n");
                break;
            }
        }
    }
    

}
/**
 * @brief receive packet
 * 
 */
void packet_rx(void)
{
    int i;
    uint16_t irqRegs = SX126xGetIrqStatus();
    uint8_t rxbuf[255];
    uint8_t size=0;
    uint8_t offset;
    SX126xClearIrqStatus(irqRegs);
    Radio.Rx(0);
    i=0;
    while (1)
    {
        irqRegs = SX126xGetIrqStatus();

        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_RX_DONE)==IRQ_RX_DONE)
        {
            printf("receive packet successfully\n");
            SX126xGetRxBufferStatus(&size,&offset);
            printf("the size is %d, the offset is %d\n",size,offset);
            SX126xGetPayload( rxbuf, &size , 255 );
            size=4;
            printf("the length of receive buffer is %d\n",size);
            for (i = 0; i < size; i++)
            {
                // printf("%c ",rxbuf[i]);
                printf("0x%02x ",rxbuf[i]);
            }
            printf("\n");
            //  SX126xGetPacketStatus(&RadioPktStatus);
            //  if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
            //  {
            //      RadioEvents->RxDone(RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt);
            //  }
            break;
        }
        else
        {
            break;//weijunfeng 20210520
            i++;
            HAL_Delay(10);
            if(i>=500)
            {
                printf("receive timeout\n");
                break;
            }
        }
    }
    


}

void register_test(void)
{
    //write register
    char xx[20];
    int a;
    printf("register write and read test\n");
    SX126xWriteRegister(0x06C0, 0x12);
    SX126xWriteRegister(0x06C1, 0x34);
    SX126xWriteRegister(0x06C2, 0x56);
    SX126xWriteRegister(0x06C3, 0x78);

    a=SX126xReadRegister(0x06C0);
    sprintf(xx,"the sync 0 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C1);
    sprintf(xx,"the sync 1 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C2);
    sprintf(xx,"the sync 2 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C3);
    sprintf(xx,"the sync 3 is 0x%02x\n",a);
    printf(xx);
}

/**
 * Main application entry point.
 */
int main_PingPang( void )
{
    bool isMaster = true;
    uint8_t i;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

    Radio.SetMaxPayloadLength( MODEM_FSK, BUFFER_SIZE );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx( RX_TIMEOUT_VALUE );

    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        GpioToggle( &Led1 );

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        GpioToggle( &Led2 ); // Set LED off
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        GpioToggle( &Led1 );

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            GpioToggle( &Led2 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    }
}
