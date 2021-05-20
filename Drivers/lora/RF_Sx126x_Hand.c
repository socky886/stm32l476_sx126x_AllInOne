
#include "RF_Sx126x_Hand.h"

#include <Mysys_pic.h>
#include "../include/stdio.h"
#include "../include/string.h"
#include "si446x.h"
#include "SI446X_DEFS.H"


unsigned char ItStatus;  //????????��?
unsigned char ItStatus1;
volatile unsigned char RSSI;

static tRFLRStates RFLRState = RFLR_STATE_IDLE;      //LORA??????????????SX1276Process???????

UINT16   RF_Dev_INFO;         //????��???????
Uint8    RF4463_RSSI_buf[4];

volatile  Uint8  RF_test0=0;
volatile  Uint8  RF_test1=0;

// Default settings
tFSKSettings FSKSettings =
{
    199,    //58,              // Channel (RFFrequency)
    127,                // Power  ???????
};



void  RF446x_IOset(void);
Uint8 Si4463_Working_Condition(void);   //��???????????

void  RF446x_IOset(void)
{
 RF_SDO_TRIS=Potr_In;     //?څ????
 RF_SDI_TRIS=Potr_Out;    //?څ???
 RF_SDI=0;
 RF_SCK_TRIS=Potr_Out;
 RF_SCK=0;
 MCU_NSEL_TRIS=Potr_Out;
 MCU_NSEL=1;             //????? ????��
 EZRP_SDN_TRIS=Potr_Out;
 EZRP_SDN=1;
 EZRP_NIRQ_TRIS=Potr_In;
 EZRP_NIRQ_IE=0;
 EZRP_NIRQ_IES=0;       //????????��
 EZRP_NIRQ_IFG=0;
 RF_Dev_INFO=0;          //��????
 RSSI=0;

 }


void RF446x_Init(void)
{
   UINT8  Rf_Tem_Buf[10];
   Rf_Hand_str.Rf_Hadrware_ini=false;
   RF446x_IOset();
   SI446X_RESET( );        //SI446X ??�`��
   SI446X_CONFIG_INIT( );  //SI446X ??????????��???
   
   SI446X_PART_INFO( &Rf_Tem_Buf[0] );     //???��???
   RF_Dev_INFO= (((UINT16)Rf_Tem_Buf[2])<<8)+Rf_Tem_Buf[3];
   
   if(RF_Dev_INFO==0x4463)
   Rf_Hand_str.Rf_Hadrware_ini=true;
   NOP();
 }

void SI4463_RESET(void)
{
 SI446X_RESET( ); 
 }

//??Rf_Rec_Str.Rf_Buffer?��??????Rf_Bus??
void SI4463GetRxPacket( void *buffer, uint16_t size )
{
    uint16_t RxPacketSize;
    RxPacketSize=size ;
    memcpy( buffer, Rf_Rec_Str.Rf_Buffer, ( size_t )RxPacketSize );
}


//??Rf_Bus????????Rf_Tec_Str.Rf_Buffer????????
void SI4463SetTxPacket( const void *buffer, uint16_t size )
{
    uint16_t TxPacketSize;
    TxPacketSize = size;
    memcpy( Rf_Tec_Str.Rf_Buffer, buffer, ( size_t )TxPacketSize ); 
    
    SI446xFSKSetRFState(RFLR_STATE_TX_INIT );  
}




//?څLORA??????????
void SI446xFSKSetRFState( uint8_t state )
{
    RFLRState = state;
} 

uint8_t SI446xFSKGetRFState(void)
{
    return RFLRState;
} 
// ?څ???????��??
void SI446xFSKSetTX_Power(uint8_t pW)
{
 SI446X_SET_PA_PWR_LVL(pW);
 }

// ???????????????????��?I/O	
void SX4463_Interupt(void)
	{           
    if(EZRP_NIRQ_IFG)                                      //?????????��?  
//    if(EZRP_NIRQ)
       {            
          SI446X_INT_STATUS(Rf_Rec_Buf);    //?????????0   
         ItStatus1=Rf_Rec_Buf[3];          //??????????
         
         SI446X_GET_MODEM_STATUS(Rf_Rec_Buf);       //???RSSI
         RSSI=Rf_Rec_Buf[3];    //?????RSSI
         if(ItStatus1&( 1<<4 ))              //?????��?
	   {
            NOP();
            Rf_Rec_Str.Rf_Buffer_All_Len=SI446X_READ_PACKET(Rf_Rec_Str.Rf_Buffer,Rf_Rec_Str.Rf_Buffer_All_Len);     //???????
            Disable_RF_IE();
            RFLRState=RFLR_STATE_RX_DONE;   //???��??????                
	     }
         else if (ItStatus1&( 1<<5 ))       //?????��?
          {
            RFLRState=RFLR_STATE_TX_DONE;   //???��??????
         }
        else if(ItStatus1&( 1<<3 ))   //CRC????
         {
          NOP();
          RFLRState =RFLR_STATE_CRC_ERROR;       
	 }
        
        EZRP_NIRQ_IFG=0;
      }    
     
  }
	

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint8_t SI446xFSKProcess( void )
{
    uint8_t result = RF_BUSY;  //????
    
    switch( RFLRState )
    {
    case RFLR_STATE_IDLE:
        result = RF_IDLE;           //???????
        break;
    case RFLR_STATE_RX_INIT:     //????????
        SI446X_INT_STATUS(Rf_Rec_Buf);    //????��???
        si4463_SetFiled2Length(63);                //??????? ???????????????
        SI446X_START_RX(FSKSettings.Channel ,0 , PACKET_LENGTH,8,8,8 );
        RFLRState = RFLR_STATE_RX_RUNNING ;                    //?????????
        EnableA_T(5)=1;                                        //2S?????????????????
        StartA_T(5)=0;
        EndA_T(5)=0; 
        EnableA_T(7)=0;
        StartA_T(7)=0;
        EndA_T(7)=0; 
        Clr_RF_INTF();
        Enable_RF_IE();             //????????��?   //???result ???????BUSY
        break;
    case RFLR_STATE_RX_RUNNING:     //????????
         if((EnableA_T(5)&&EndA_T(5))&&(!Sens_Test_Enable))     //???????????????
        {
        EnableA_T(5)=0;
        result =RF_RX_TIMEOUT;          //???????
         }
        else                            //??��?????????????????��
        result = RF_IDLE;           //???????
        break;                               //???result ???????BUSY
    case RFLR_STATE_RX_DONE:        //???????
         NOP();
        EnableA_T(5)=1;                                        //2S?????????????????
        StartA_T(5)=0;
        EndA_T(5)=0; 
        result = RF_RX_DONE;        //???????????
        break;
    case RFLR_STATE_RX_TIMEOUT:          //??????
        NOP();
        result = RF_RX_TIMEOUT;
        break;
    case RFLR_STATE_TX_INIT:
        si446x_change_state(READY);
        SI446X_TX_FIFO_RESET();
        si4463_SetFiled2Length(Rf_Tec_Str.Rf_Buffer_All_Len);                    //??????????
        SI446X_W_TX_FIFO( Rf_Tec_Str.Rf_Buffer,  Rf_Tec_Str.Rf_Buffer_All_Len);    //????????��??????
	    SI446X_INT_STATUS(Rf_Rec_Buf);    //????��???
    	//???????????��????????????????????��???????
          NOP(); 
        EnableA_T(7)=1;       //????????12,???��?????????
        StartA_T(7)=0;
        EndA_T(7)=0;
        EnableA_T(5)=0;                                        //2S?????????????????
        StartA_T(5)=0;
        EndA_T(5)=0; 
        Con_Tim=0;
        //The radio forms the packet and send it automaticall
        SI446X_START_TX(FSKSettings.Channel, 0x30, Rf_Tec_Str.Rf_Buffer_All_Len);      //????????0x30
//        SI446X_START_TX(FSKSettings.Channel, 0x80, Rf_Tec_Str.Rf_Buffer_All_Len);    
        
        Clr_RF_INTF();
        Enable_RF_IE();              //?????????��?
        RFLRState = RFLR_STATE_TX_RUNNING;
        break;                       //???result ???????BUSY
    case RFLR_STATE_TX_RUNNING:      //?????????????????��????
        if((EnableA_T(7))&&(EndA_T(7)))
        result = RF_TX_TIMEOUT;      //??????
        break;                       //???result ???????BUSY
    case RFLR_STATE_TX_DONE:         //??????????��?
        EnableA_T(7)=0;
        StartA_T(7)=0;
        EndA_T(7)=0;                 //?????????????
        EnableA_T(6)=1;              //?????????????????��??????��?????
        StartA_T(6)=0;
        EndA_T(6)=0; 
        
//        EnableA_T(5)=1;              //
//        StartA_T(5)=0;
//        EndA_T(5)=0; 
//        RFLRState = RFLR_STATE_RX_RUNNING; 
        RFLRState = RFLR_STATE_RX_INIT;   //????????
        result = RF_TX_DONE;    
        break;
    case RFLR_STATE_CAD_INIT:           //????????
        NOP();
        break;
    case RFLR_STATE_CAD_RUNNING:
        NOP();
        break;
    case RFLR_STATE_CAD_DETECTED:
        NOP();
        break;
    case RFLR_STATE_CRC_ERROR:
        NOP();
        RFLRState = RFLR_STATE_RX_INIT;
        break;
    default:
        break;
    } 
    
    if(Rf_chip_err_Status)     //????????CTS?????????????��??????��???????
    result = RF_TX_TIMEOUT;
    else if(Si4463_Working_Condition())
    result = RF_TX_TIMEOUT;   //SI4463��???????
    return result;
}


/*
 * si4463 ��??????????? 
 * ????0????????????��???
 */
Uint8 Si4463_Working_Condition(void)
{
    Uint8  Working_State=0;
    EnableA_T(9)=1;               //????????
   if(EnableA_T(9)&&EndA_T(9))
   {
    Uint8  SI446X_Buf[3]={0}; 
    Uint8  Back_flag=0;
    Back_flag=EZRP_NIRQ_IE;
//    DisAllINT();
    Disable_RF_IE();
    SI446X_REQUEST_DEVICE_STATE( SI446X_Buf );
    EZRP_NIRQ_IE=Back_flag;   //??????????????��?????????
   
    
    if((((RFLRState==RFLR_STATE_RX_RUNNING)||(RFLRState==RFLR_STATE_RX_DONE))&&((SI446X_Buf[1]&0x0F)!=RX_State))||
      ((RFLRState==RFLR_STATE_TX_RUNNING)&&((SI446X_Buf[1]&0x0F)!=TX_State))||
      (!(SI446X_Buf[1]&0x0F))||(SI446X_Buf[2]!=FSKSettings.Channel))
      Working_State = RF_TX_TIMEOUT;
    EnableA_T(9)=1;               //
    StartA_T(9)=0;
    EndA_T(9)=0;
   
   }
    
    return Working_State;
}

