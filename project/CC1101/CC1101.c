
/*******************************************************************************
            Qitas wirless sensor for CC1101 (STM8)
*******************************************************************************/
#include <stdio.h>
#include "regs.h"
#include  "CC1101.h"

#define  PKT_LEN   60

#define CSn_LOW   GPIO_ResetBits(SPI_PORT, CS_Pin)
#define CSn_HIGH  GPIO_SetBits(SPI_PORT, CS_Pin)

void  halSpiStrobe(BYTE strobe);
void  halRfWriteReg(BYTE addr, BYTE value);
void  halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count);

/*******************************************************************************
* Function Name  : CC1101_Init
*******************************************************************************/
void CC1101_Init(void)
{
	RESET_CC1101();		
	WriteRfSettings();
	//RF_SetPKT(0,PKT_LEN);
	//RF_SetGDO(1);
	RF_SetPWR(0x19);
	RF_SetADR(0x22);
	//RF_SetADR(MAIL[2]);
	RF_TX();
}
    
/*******************************************************************************
* Function Name  : RF_SetPA
*******************************************************************************/
void RF_SetPWR(u8 cofig)
{
        u8 LS,HS;
	u8 paTable[10]= {0x03,0x17,0x1D,0x26,0x37,0x50,0x86,0xCD,0xC5,0xC0}; 
	HS=cofig>>4;
	LS=(cofig & 0x0F);
  	if(HS<1) HS=1;
	halSpiWriteBurstReg(PATABLE, paTable+LS, HS);   
	if(HS>1)  halRfWriteReg(FREND0,   0x17);
}

/*******************************************************************************
* Function Name  : WriteRfSettings
*******************************************************************************/
void WriteRfSettings(void) 
{

    halRfWriteReg(FSCTRL1,  0x06);//FSCTRL1   Frequency synthesizer control.
    halRfWriteReg(FREQ2,    0x10);//FREQ2     Frequency control word, high byte.
    halRfWriteReg(FREQ1,    0xa7);//FREQ1     Frequency control word, middle byte
    halRfWriteReg(FREQ0,    0x62);//FREQ0     Frequency control word, low byte
	
    halRfWriteReg(CHANNR,   0x00);//CHANNR    Channel number.                     ------default value
    halRfWriteReg(DEVIATN,  0x15);//250k   Modem deviation setting (when FSK modulation is enabled).
   
    halRfWriteReg(MDMCFG4,  0xf6);//MDMCFG4   Modem configuration
    halRfWriteReg(MDMCFG3,  0x83);//MDMCFG3   Modem configuration.
    halRfWriteReg(MDMCFG2,  0x03);//0b, MDMCFG2  Modem configuration.
	
    halRfWriteReg(MDMCFG1,  0xa2);//MDMCFG1   Modem configuration.
    halRfWriteReg(MDMCFG0,  0xf8);//MDMCFG0   Modem configuration. 
	
    halRfWriteReg(FREND1,   0x56);//FREND1    Front end RX configuration
    halRfWriteReg(FREND0,   0x10);//FREND0    Front end RX configuration         ------default value
    halRfWriteReg(IOCFG0,   0x07);
    halRfWriteReg(FIFOTHR,     1);
    halRfWriteReg(MCSM1,    0x3A); 
    halRfWriteReg(MCSM0 ,   0x18);//MCSM0     Main Radio Control State Machine configuration
    halRfWriteReg(FOCCFG,   0x16);//FOCCFG    Frequency Offset Compensation Configuration
    halRfWriteReg(BSCFG,    0x6c);//BSCFG     Bit synchronization Configuration.   ------default value
    halRfWriteReg(AGCCTRL2, 0x03);//AGCCTRL2  AGC control                          ------default value
    halRfWriteReg(AGCCTRL1, 0x40);//AGCCTRL1  AGC control                          ------default value
    halRfWriteReg(AGCCTRL0, 0x91);//AGCCTRL0  AGC control.                       ------default value
    halRfWriteReg(FSCAL3,   0xe9);//FSCAL3    Frequency synthesizer calibration.
    halRfWriteReg(FSCAL2,   0x2a);//FSCAL2    Frequency synthesizer calibration. ------default value
    halRfWriteReg(FSCAL1,   0x00);//FSCAL1    Frequency synthesizer calibration.
    halRfWriteReg(FSCAL0,   0x1f);//FSCAL0    Frequency synthesizer calibration.
    halRfWriteReg(FSTEST,   0x59);
    halRfWriteReg(TEST2,    0x81);
    halRfWriteReg(TEST1,    0x35);
    halRfWriteReg(TEST0,    0x09);
    halRfWriteReg(PKTCTRL0, 0x44);  //Fixed packet length mode. 
    halRfWriteReg(PKTLEN,     60);
}

/*******************************************************************************
* Function Name  : RF_SetAddr
*******************************************************************************/
void RF_SetADR(u8 addr)
{ 
    halRfWriteReg(ADDR, addr);  
    if(MAIL[0]!=0 && MAIL[1]!=0 && MODE[0]!=PAIR_MODE)
    {
	  		
	    //halRfWriteReg(SYNC1, MAIL[0]);          
	    //halRfWriteReg(SYNC0, MAIL[1]);       
    }
    else
    {		
	    //halRfWriteReg(SYNC1, 0x59);          
	    //halRfWriteReg(SYNC0, 0x4A); 
    }
   if(addr!=0 && addr!=0XFF)  halRfWriteReg(PKTCTRL1, 0x0D);
   else   halRfWriteReg(PKTCTRL1, 0x0C);   
    //halRfWriteReg(PKTCTRL1, 0x0C);   
}

/*******************************************************************************
* Function Name  : RF_SetMode
*******************************************************************************/
void RF_GetTemp(void)
{ 
	halRfWriteReg(PTEST, 0xBF);//enable
	halRfWriteReg(IOCFG0,0x86);
	halRfWriteReg(PTEST, 0x7F);//disable
}
/*******************************************************************************
* Function Name  : halSpiByteWrite
* Description    : send 8 bit data to SDO line
*******************************************************************************/
BYTE halSpiByteWrite(BYTE cData)
{  
	CSn_LOW;
	while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);
	Delay_us(2);
	SPI_SendData(SPI1,cData);
	Delay_us(4);
	if(SPI_GetFlagStatus(SPI1,SPI_FLAG_RXNE) == RESET);
	return SPI_ReceiveData(SPI1);	 
}

/*******************************************************************************
* Function Name  : halRfWriteReg
*******************************************************************************/

void halSpiStrobe(BYTE strobe)
{
    BYTE SPI_DATA;
    CSn_LOW;
    SPI_DATA = strobe;
    halSpiByteWrite(SPI_DATA);
    CSn_HIGH;
}

/*******************************************************************************
* Function Name  : halRfWriteReg
*******************************************************************************/
BYTE halSpiReadStatus(BYTE addr)
{
    BYTE SPI_DATA;
    CSn_LOW;
    SPI_DATA = (addr | READ_BURST);
    halSpiByteWrite(SPI_DATA);
    SPI_DATA = halSpiByteWrite(Dummy);
    CSn_HIGH;
    return SPI_DATA;
}

/*******************************************************************************
* Function Name  : halRfWriteReg
*******************************************************************************/
void halRfWriteReg(BYTE addr, BYTE value)
{
    BYTE SPI_DATA;
    CSn_LOW;
    SPI_DATA = addr;
    halSpiByteWrite(SPI_DATA);
    SPI_DATA = value;
    halSpiByteWrite(SPI_DATA);
    CSn_HIGH;
}

/*******************************************************************************
* Function Name  : halSpiReadReg
*******************************************************************************/
BYTE halSpiReadReg(BYTE addr)
{
    BYTE SPI_DATA;
    CSn_LOW;
    SPI_DATA = (addr | READ_SINGLE);
    halSpiByteWrite(SPI_DATA);
    SPI_DATA = halSpiByteWrite(Dummy);
    CSn_HIGH;
    return SPI_DATA;
}

/*******************************************************************************
* Function Name  : halSpiReadBurstReg
*******************************************************************************/
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
{
    BYTE i, SPI_DATA;
    CSn_LOW;
    SPI_DATA = (addr | READ_BURST);
    halSpiByteWrite(SPI_DATA);
    for (i = 0; i < count; i++)
    {
        SPI_DATA = halSpiByteWrite(Dummy);
        buffer[i] = SPI_DATA;
    }
    CSn_HIGH;
}
/*******************************************************************************
* Function Name  : halSpiWriteBurstReg
*******************************************************************************/
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
{
    BYTE i, SPI_DATA;
    CSn_LOW;
    SPI_DATA = addr | WRITE_BURST;
    halSpiByteWrite(SPI_DATA);
    for (i = 0; i < count; i++)
    {
        SPI_DATA = buffer[i];
        halSpiByteWrite(SPI_DATA);
    }
    CSn_HIGH;
}
/*******************************************************************************
* Function Name  : RESET_CC1101
*******************************************************************************/
void RESET_CC1101()
{
    CSn_HIGH;
    Delay_us(3);
    CSn_LOW;
    Delay_us(3);
    CSn_HIGH;
    Delay_us(45);
    CSn_LOW;
    halSpiByteWrite(SRES);
    CSn_HIGH;
}
/*******************************************************************************
* Function Name  : RF_RX
*******************************************************************************/
void RF_TX(void)
{
   //CSn_LOW;
   halSpiStrobe(SFTX);
   Delay_us(20);
   halSpiStrobe(SIDLE);
   Delay_ms(1);
   halSpiStrobe(STX);
   Delay_ms(1);
   //CSn_HIGH;
}
/*******************************************************************************
* Function Name  : halRfSendPacket
*******************************************************************************/
void halRfSendPacket(BYTE *txBuffer, u8 size)
{
//    halSpiStrobe(SIDLE);
//    halSpiStrobe(SFTX);
//    Delay_ms(1); 
    halSpiStrobe(STX); 
    Delay_ms(1);
    halSpiWriteBurstReg(TXFIFO, txBuffer, size);  
    Delay_ms(100);
}
/*******************************************************************************
* Function Name  : RF_RX
*******************************************************************************/
void RF_RX(void)
{
   //CSn_LOW;
   halSpiStrobe(SFRX);
   Delay_us(20);
   halSpiStrobe(SIDLE);
   Delay_us(900);
   halSpiStrobe(SRX);
   Delay_us(900);
   //CSn_HIGH;
}

/*******************************************************************************
* Function Name  : RF_RX
*******************************************************************************/
void RF_END(void)
{
   //CSn_LOW;
   halSpiStrobe(SIDLE);
   Delay_us(100);
   halSpiStrobe(SXOFF);
   Delay_us(900);
   //CSn_HIGH;
}
/*******************************************************************************
* Function Name  : RF_RX
*******************************************************************************/
u8 RF_CHCK(void)
{
     u8 check[3]={0};
     check[0] = halSpiReadReg(TEST0);
     check[1] = halSpiReadReg(TEST1);
     check[2] = halSpiReadReg(TEST2);
     if(check[0]==0x09 && check[1]==0x35 &&check[2]==0x81) return BACK_SUCCESS;
     return BACK_ERROR;
}
/*******************************************************************************
* Function Name  : halRfReceivePacket
*******************************************************************************/
BOOL GetPacket(void) 
{	
    if((halSpiReadStatus(RXBYTES) & NUM_RXBYTES))
    {
          halSpiReadBurstReg(RXFIFO, RxBuf, PKT_LEN);		  	       
	  return BACK_SUCCESS;
    }
    return BACK_ERROR;
}
/*******************************************************************************
* Function Name  : halRfSendPacket
*******************************************************************************/
int GetRSSI(void)
{
     	int rssi_dec; 
	 int rssi_dBm; 
	 u8 rssi_offset = 74; 
	 rssi_dec = halSpiReadStatus(RSSI); 
	if (rssi_dec >= 128) rssi_dBm = (long)((long)( rssi_dec - 256) / 2) - rssi_offset; 
	else rssi_dBm = (rssi_dec / 2) - rssi_offset;	
	return rssi_dBm;		  
}

/*******************************************************************************
* Function Name  :  External IT PIN4 Interrupt routine.
*******************************************************************************/
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
	//Delay_us(100);
   	GetPacket();
 	EXTI_ClearITPendingBit(EXTI_IT_Pin4);
	
}
	


