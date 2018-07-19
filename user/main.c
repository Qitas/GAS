


/*******************************************************************************
* Description    :STM8S103F3P3  TSSOP20 
*******************************************************************************/
#include "DHT11.h"
#include "bh1750.h"

u16 Get_AD(u8 chan);
u16 lx,AD_U,AD_M,AD_D;
u8 mode,Pointer;
/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
void HAL_Init(void) 
{
      asm("sim"); 
      CLK_CKDIVR = 0x00;
      Delay_ms(2); 
      
      PA_DDR=0X06;
      PA_CR1=0X00;
      PA_CR2=0X00;
      PA_ODR=0X06;
   
      PB_DDR=0X30;
      PB_CR1=0X30;
      PB_CR2=0X30;
      PB_ODR=0X30;
      
      PC_DDR=0X00;
      PC_CR1|=0XFF;
      PC_CR2|=0X00;
      //PC_ODR =0XFF;
      
      PD_DDR|=0X00;
      PD_CR1|=0XFF;
      PD_CR2|=0X00;
      //PD_ODR =0X0F;  
      
      CLK_PCKENR1 = 0x09;
      CLK_PCKENR2 = 0x08;
      UART1_BRR2 = 0x03;
      UART1_BRR1 = 0X68; // 16m/9600
      UART1_CR2  = 0X2C;
 
      
//      IWDG_KR =0X55;
//      IWDG_PR = 0X06;//1/256 f
//      IWDG_RLR = 0xFF;
//      IWDG_KR =0;
//      IWDG_KR = 0XAA;
      //IWDG->KR = 0XCC;
      asm("rim"); 
}
/*******************************************************************************
* Function Name  : main
* Description    : 
*******************************************************************************/

void main( void )
{  
    u8 buff,cnt;
    HAL_Init();  
    Pointer=MODE[1];
    if(Pointer>=0x7F)
    {
        Pointer=MODE[2];
    	EEPROM_B(HEAD_ADDR+2,Pointer+1);
	Pointer=0;
    }
    else Pointer++;
    EEPROM_B(HEAD_ADDR+1,Pointer);
     
    while(1)
    {
//      lx=Convert_BH1750();
//	if(lx<1000 && MODE[0]!=NIGHT)         EEPROM_B(HEAD_ADDR,NIGHT); 
//	else if(lx>1500 && MODE[0]!=SUNNY)   EEPROM_B(HEAD_ADDR,SUNNY); 
//	else if(MODE[0]!=CLOUDY)             EEPROM_B(HEAD_ADDR,CLOUDY); 
	
	 buff=Get_AD(2);
	 if(buff<IN[Pointer])  EEPROM_B(STORE_ADDR+cnt,buff);

	for(cnt=2;cnt<8;cnt++) 
	{	     	     
	      buff=Get_AD(cnt);
	      //if(buff<IN[Pointer])  EEPROM_B(STORE_ADDR+cnt,buff);
	      Delay_ms(100);
	      //UASRT_SEND(buff);
	      //EEPROM_B(STORE_ADDR+cnt,PD_IDR);
	}
	 //    DHT11();
//	buff=Get_AD(7) ;
//	//UASRT_SEND(buff);
	
	
//	UASRT_SEND(LX[0]);
//	UASRT_SEND(LX[1]);
	Delay_ms(1000);
	if(Pointer==0) break;	
	
	CLK_SWR = 0x00;
	CLK_SWCR = 0x00;

    }
    WWDG_CR = 0x80;
    WWDG_CR &= 0xBF;
}


/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
u16 Get_AD(u8 chan) 
{
      u16 buff=0;
      ADC_CR1=0X01;
      ADC_CR2=0X00;
      ADC_CSR=chan;
      ADC_CR1|=0X01;
      while((ADC_CSR & 0X80)==0);
      ADC_CR1 &=0XFE;     
      buff=(ADC_DRH<<2);
      buff+=ADC_DRL;
      return buff;
}

/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
void UASRT_SEND(u8 data) 
{
      while((UART1_SR & 0x80) == 0x00); 
      UART1_DR = data;
}
//#pragma  message("hi word")
#pragma vector=UART1_R_RXNE_vector
__interrupt void UART1_RX_RXNE(void)
{
      u8 c;
      c = UART1_DR; 
      while(!UART1_SR_TXE);
      switch(c)
      {
      case 1:;
      }
      c++; 
      UASRT_SEND(c) ;
}
/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
void SERVO(u8 angle) 
{
  
     
}
/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
void IR(u8 Plant) 
{

}

/*******************************************************************************
* Function Name  : Time
*******************************************************************************/
void Delay_ms(uint16_t i) 
{
	uint16_t x=1440;  
	while(i--) 
	{
	    while(x--);
	    x=1440; 
	}
}
void Delay_us(uint16_t i) 
{ 
	while(i--);
}
/*******************************************************************************
* Function Name  : EEPROM_W
*******************************************************************************/

u8 EEPROM_B(u16 addr,u8 data)
{
      do{
	   FLASH_DUKR = 0xAE;
	   FLASH_DUKR = 0x56;	  	 
      }while((FLASH_IAPSR & 0x08) == 0);
      
      *(u8*)addr =data;  

     while((FLASH_IAPSR & 0x04) == 0);
     FLASH_IAPSR &= 0xF7;
     return  *(u8*)addr ; 
}

/*******************************************************************************
* Function Name  : 
* Description    : 
*******************************************************************************/
void BEEP(u8 data) 
{
      CLK_PCKENR1 |= 0x08;
      BEEP_CSR=0X1E;//32*8
      BEEP_CSR|=0X20;

}
