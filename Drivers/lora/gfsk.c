#include "gfsk.h"
#include "board-config.h"
#include "sx126x_all.h"
#include "ww_radio.h"

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

void fsk_packet_init(void)
{
    //GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    //GpioInit(&Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    SpiInit(&SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC);
    SX126xIoInit();

    // init
    sx126x_dev_init();
}
void fsk_tx(void)
{
    int i;
    uint16_t irqRegs;
    

    Buffer[0] = 0x12;
    Buffer[1] = 0x23;
    Buffer[2] = 0x45;
    Buffer[3] = 0x67;
    Buffer[4] = 0xff;
    Buffer[5] = 0x00;
    Buffer[6] = 0xf0;

    // We fill the buffer with numbers for the payload
    BufferSize=4;
    
    SX126xSetStandby( STDBY_RC );
    
    DelayMs(2);
    irqRegs=SX126xGetIrqStatus();
    SX126xClearIrqStatus(irqRegs);
    // SX126xClearIrqStatus(0xffff);
    
    sx126x_dev_Send(Buffer, BufferSize);
    // SX126xClearIrqStatus(0xffff);
    i=0;
    while (1)
    {
        irqRegs=SX126xGetIrqStatus();
        printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_TX_DONE) ==IRQ_TX_DONE)
        {
            printf("transmit finished\n");
           
            break;
        }
        else
        {
           
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

void fsk_rx(void)
{
    int i;
    uint16_t irqRegs = SX126xGetIrqStatus();
    uint8_t rxbuf[255];
    uint8_t size=0;
    uint8_t offset;
    SX126xClearIrqStatus(irqRegs);
    sx126x_dev_Rx(0);
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

