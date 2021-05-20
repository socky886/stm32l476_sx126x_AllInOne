#ifndef _RF_SX126X_HAND_H_
#define _RF_SX126X_HAND_H_

//---------------RF446x���ÿڶ���-----------------------


#define RF_SDO_TRIS         TRISAbits.TRISA0
#define	RF_SDO              PORTAbits.RA0         //SPI�������� 
#define RF_SDI_TRIS         TRISAbits.TRISA1
#define RF_SDI              LATAbits.LATA1	      //SPI�������
#define RF_SCK_TRIS         TRISAbits.TRISA2
#define	RF_SCK              LATAbits.LATA2          //SPIʱ������˿�
#define MCU_NSEL_TRIS       TRISAbits.TRISA3
#define	MCU_NSEL            LATAbits.LATA3          //SPIƬѡ
#define EZRP_SDN_TRIS       TRISAbits.TRISA4
#define EZRP_SDN            LATAbits.LATA4          //SDN=1ʱ�ر�оƬ����	,SDN=0ʱоƬ��������
#define EZRP_NIRQ_TRIS       TRISBbits.TRISB0
#define EZRP_NIRQ            PORTBbits.RB0 
#define EZRP_NIRQ_IE         INTCONbits.INT0IE            //��������Ч
#define EZRP_NIRQ_IES        INTCON2bits.INTEDG0
#define EZRP_NIRQ_IFG        INTCONbits.INT0IF

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
}tRFLRStates;            //LORA�����߹���״̬


extern unsigned char ItStatus;  //���ͽ����ж�
extern unsigned char ItStatus1;
extern UINT16  RF_Dev_INFO;         //����оƬ�ͺ���Ϣ
extern volatile unsigned char RSSI;
extern tFSKSettings FSKSettings;

void RF446x_Init(void);
void SI4463_RESET(void);
void SI446xFSKSetRFState( uint8_t state );
void SI446xFSKSetTX_Power(uint8_t pW);  //�趨���书��
uint8_t SI446xFSKGetRFState(void);
void SI4463GetRxPacket( void *buffer, uint16_t size );
void SI4463SetTxPacket( const void *buffer, uint16_t size );
void SX4463_Interupt(void);
uint8_t SI446xFSKProcess( void );
    

#endif