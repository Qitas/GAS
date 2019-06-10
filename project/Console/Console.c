
/*******************************************************************************
                      Qitas Wirless Sensor For STM8L151
*******************************************************************************/
#define  VERSION           3
#define  SERIAL         0xAB

char *DAY="2017_6_12_TQ";

/*******************************************************************************
* INCLUDE
*******************************************************************************/

#include "tlv.h"
#include "SI7021.h"
#include "Console.h"

uint8_t TxBuf[61]={0};
uint8_t RxBuf[61]={0};
u8   FLAG_MODE;
u8   FLAG_TIME;
u8   FLAG_RSSI;
u8   FLAG_CHEC;
/*******************************************************************************
* Function Name  : Delay
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
* Function Name  :Power_ON
*******************************************************************************/
void HAL_INIT(void)
{ 
	CLK->ICKCR |= 0x01;
	while( CLK->ICKCR & 0x02 == 0) 
	{
	    FLAG_TIME++; 
	    if(FLAG_TIME>100) 
	    {
		  SYS_Status|=0x80;		  
		  break;
	    }
	}
	CLK->CKDIVR = 0;
	if(FLAG_TIME!=M[4])  EEPROM_B(M_ADDR+4,FLAG_TIME);
	
	GPIOA->CR2=0X20;
	GPIOA->CR1=0XFF;
	GPIOA->DDR=0XDF;
	GPIOA->ODR=0x00;
	
	GPIOB->CR2=0X60;
	GPIOB->CR1=0XFF;
	GPIOB->DDR=0X7F;
	GPIOB->ODR=0x1C;
	
	GPIOC->CR2=0X00;
	GPIOC->CR1=0XFF;
	GPIOC->DDR=0XFF;
	GPIOC->ODR=0x03;
	
	GPIOD->CR2=0X10;
	GPIOD->CR1=0XEF;
	GPIOD->DDR=0XEF;
	GPIOD->ODR=0x00;
	
	CLK->PCKENR1 =0x10; 		
	SPI1->CRCPR = 0x07;
	SPI1->CR2 = 0x03;
	SPI1->CR1 =0x54; 
	
	CLK->PCKENR2 |=0x01; 
	ADC1->CR1 &=0x64;
	ADC1->CR2 &=0x80;
		 
	CC1101_Init();
	EXTI->CR2 =  0xF1;
	enableInterrupts();
}

/*******************************************************************************
* Function Name  :Power_ON
*******************************************************************************/
int Power_ON(void)
{ 
  
        HAL_INIT(); 
	
#ifdef SERIAL	
	//SN_code(1);   
	//EEPROM_B(MODE_ADDR,WORK_MODE); 
	if(SSN[1]!=SERIAL && SSN[9]!=SERIAL)   FLUSH(1); 
#endif
 	if(MODE[0]==PAIR_MODE || MODE[0]==WORK_MODE || MODE[0]==CHCK_MODE)  FLAG_MODE=MODE[0];
        else  FLAG_MODE=MODE[1];
	if(0!=FLAG_MODE) 
	{    	      
	      for(FLAG_TIME=0;FLAG_TIME<8;FLAG_TIME++)
	      { 
		  if(SSN[FLAG_TIME]==SSN[FLAG_TIME+8]) FLAG_CHEC++;
	      }
	      if(FLAG_CHEC==8 &&  PAIR_MODE!=FLAG_MODE) 
	      {
		  RF_TX();
		  if(MODE[3]!= FLAG_SEND) EEPROM_B(MODE_ADDR+3,FLAG_SEND);
	          return  BACK_SUCCESS;
	      }
	      else if(FLAG_CHEC==8 && PAIR_MODE==FLAG_MODE)  
	      { 
		  RF_RX();
		  FLAG_TIME=MODE[2];
		  if(FLAG_TIME==0xFF) FLAG_TIME=5;
		  EEPROM_B(MODE_ADDR+2,FLAG_TIME+1);
		  return  BACK_SUCCESS;
	      }	     
	      for(FLAG_TIME=100;FLAG_TIME>0;FLAG_TIME--) { LED_PORT->ODR ^= 0x04;Delay_ms(80);}
	}		
	Power_OFF();	
        return  BACK_ERROR;
}

/*******************************************************************************
* Function Name  :Power_OFF
* Description    :
*******************************************************************************/
void Power_OFF(void)
{
     u8 cnt=100;
     RF_END();
     while(cnt--)
     { 
	 if((cnt%2)==0)  GPIOD->ODR ^=0X20;
	 Delay_us(100);
     }
    if(MODE[10]!=DEBUG_MODE)
    {
	SYS_Status |= 0x40; 
	STORE_STAT();	  
    }
}

/*******************************************************************************
* Function Name  :SN_code
*******************************************************************************/
#ifdef SERIAL
void SN_code(u8 sn)
{
  	u8 cnt;
	for(cnt=0;cnt<16;cnt++) TxBuf[cnt]=SERIAL;
	if(SSN[0]!=0 && sn==0)  {TxBuf[0]=SSN[0];TxBuf[8]=SSN[8];}
	else  { TxBuf[0]=sn; TxBuf[8]=sn;}
	TxBuf[16]=SSN[16]+1;
	//TxBuf[16]=0;
	FLASH_W(SSN_ADDR,TxBuf,17);	
}	
/*******************************************************************************
* Function Name  :FLUSH
*******************************************************************************/
void FLUSH(u8 SN)
{
	LED_PORT->ODR ^= 0x0c;
	FLASH_Flush(DATA_HEAD,0);
	LED_PORT->ODR ^= 0x0c;
	SN_code(SN);
	LED_PORT->ODR ^= 0x0c;
	EEPROM_Flush(MODE_ADDR,0);		
}
#endif

/*******************************************************************************
* Function Name  : MODE_Mark
*******************************************************************************/
u8 MODE_Mark(u8 mode)
{
   	u8 buff;
	if(mode!=MODE[0])
	{
	    buff=MODE[0];
	    if(mode!=MODE[0])  EEPROM_B(MODE_ADDR,mode); 
	    if(buff!=MODE[1])  EEPROM_B(MODE_ADDR+1,buff); 	
	    return BACK_SUCCESS;	
	}
	return BACK_ERROR;
}
/*******************************************************************************
* Function Name  :External IT PIN5 Interrupt routine.
*******************************************************************************/
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
      Delay_ms(10);
      if((KEY_PORT->IDR & KEY_Pin)==0)
      {
	    if(FLAG_MODE!=PAIR_MODE)
	    {
		  LED_PORT->ODR &=0xFB;
		  MODE_Mark(PAIR_MODE);
		  FLAG_TIME=M[6];
		  if(FLAG_TIME<0xFF)  EEPROM_B(M_ADDR+6,FLAG_TIME+1);
		  Reset();
	    } 
      }
      EXTI_ClearITPendingBit (EXTI_IT_Pin5);
}

/*******************************************************************************
* Function Name  :Work
* Description    :
*******************************************************************************/
 
void Work(void)
{
      if(FLAG_MODE==PAIR_MODE)
      {		
	    LED_PORT->ODR &=0xFB;
	    MODE_Mark(WORK_MODE);
	    FLAG_TIME=0; 
	    while (RF_Rec(20)!=BACK_SUCCESS) 
	    { 
		if(FLAG_TIME>=8)
		{
		   FLAG_TIME=10;
		   while(FLAG_TIME--){ LED_PORT->ODR ^= 0x04;Delay_ms(100);}
		   break;
		}
		RF_RX();
	    }
	    LED_PORT->ODR |= 0x04;
	    FLAG_MODE =MODE[0];
	    RF_TX();
	    Delay_ms(600);
      }
      if(FLAG_MODE==WORK_MODE)
      {	  	    
	     Get_TLV();
	     LED_PORT->ODR &=0xF7;
	     RF_Send(MAIL[3]);
	     LED_PORT->ODR |= 0x08;
	     Data_Deal();
	     Delay_ms(100);
  //	   LED_PORT->ODR &=0xF7; 
  //	   RF_Send(MAIL[3]);
  //	   LED_PORT->ODR |= 0x08;
  //	   Delay_ms(100);	   
      }
      else  if(FLAG_MODE==CHCK_MODE)
      {	
	   LED_PORT->ODR &=0xF3; 
	   Get_TLV();		   
	   RF_Send(0x22);
	   Delay_ms(400);
	   RF_Send(0x22);
	   Delay_ms(400);
	   RF_Send(0x22);	   
	   for(FLAG_TIME=100;FLAG_TIME>0;FLAG_TIME--) { LED_PORT->ODR ^= 0x0C;Delay_ms(100);}	
	   //if(PAIR_MODE == MODE[0] && MODE[2]<2)  Reset();
      }   
     Power_OFF();
}


/*******************************************************************************
* Function Name  :Work
* Description    :
*******************************************************************************/
 
void Data_Deal(void)
{
        u8 buff;		
	if(MODE[5]==FLAG_ERROR)
	{
	    buff=M[5]+1;
	    EEPROM_B(M_ADDR+5,buff);
	}
	buff=TEMP[0];	
	if(buff>60) EEPROM_B(TEMP_ADDR,1);
	else
	{
	    EEPROM_B((TEMP_ADDR+buff+2),COUT[0]);
	    EEPROM_B(TEMP_ADDR,buff+3);
	}
	
	if(MODE[6]==FLAG_ERROR)
	{
	    buff=M[6]+1;
	    EEPROM_B(M_ADDR+6,buff);
	}
	 buff=HUMI[0];
	 if(buff>60) EEPROM_B(HUMI_ADDR,1);
	 else
	 {
	     EEPROM_B((HUMI_ADDR+buff+2),COUT[0]);
	     EEPROM_B(HUMI_ADDR,buff+3);
	 }
	
	if(COUT[0]==10) EEPROM_B(POWR_ADDR+COUT[1],result_power); 
	
	if(M[2]!=result_power)  EEPROM_B(M_ADDR+2,result_power); 
	Convert_Power(1);
	if(M[3]!=result_power)  EEPROM_B(M_ADDR+3,result_power);	
	
	if(MODE[4]!=SYS_Status && 0!=SYS_Status)  EEPROM_B(MODE_ADDR+4,SYS_Status);
}
/*******************************************************************************
* Function Name  :RTC_Config
* Description    :
*******************************************************************************/
void RTC_Config(uint16_t time)
{      
      RTC_DeInit();
      CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);     
      CLK_RTCClockConfig(CLK_RTCCLKSource_HSI, CLK_RTCCLKDiv_64); 
      RTC_WakeUpClockConfig(RTC_WakeUpClock_RTCCLK_Div16); 		
      RTC_ITConfig(RTC_IT_WUT, ENABLE);
      RTC_SetWakeUpCounter(time);
      RTC_WakeUpCmd(ENABLE);
}
/*******************************************************************************
* Function Name  :RTC
*******************************************************************************/
#pragma vector=6
__interrupt void RTC_IRQHandler(void)
{ 
	FLAG_TIME++;
	RTC_ClearITPendingBit(RTC_IT_WUT); 
}

/*******************************************************************************
* Function Name  : Get_TLV
* Description    :
*******************************************************************************/
int Get_TLV(void)
{	
	DATA_TLV_S* firstTlv;
	DATA_TLV_S* nextTlv;
	u8 check;
	u8 cnt;
	u16 light;
	s16 value;
	memset(TxBuf,0,sizeof(TxBuf));	
	firstTlv = getFirstTlv(TxBuf+1);
	firstTlv->type = 1;
	firstTlv->len = 1;
	firstTlv->value[0] = 0;
	
	nextTlv = getNextTlv(firstTlv);	
	
	if(FLAG_MODE==PAIR_MODE)
	{
	     nextTlv->type = 253;
	     nextTlv->len = 8;
	     for(check=0;check<8;check++)  nextTlv->value[7-check]= SSN[check];
	}
	else if(FLAG_MODE==CHCK_MODE)
      	{
		 nextTlv->type = 2;
		 nextTlv->len = 8;
		 for(check=0;check<8;check++)  nextTlv->value[7-check]= SSN[check];
	     
		nextTlv = getNextTlv(nextTlv);
		nextTlv->type = 251;
		nextTlv->len = 14; 
		
		nextTlv->value[10]= 8; //eq
		nextTlv->value[11] =1;
		
		nextTlv->value[12]= 16; //mcu
		nextTlv->value[13] = (M[4]<3) ? 1 : 0;

		nextTlv->value[0] = 1;
		nextTlv->value[1] = 1;
		
		check = CHEC_TEMP(5);
		check += CHEC_HUMI(5);		
		nextTlv->value[2]= 4;		
		nextTlv->value[3] = (check<0x40) ? 1 : 0;
		
		Convert_Power(1); 		
		check = result_power;		
		nextTlv->value[4]= 5;
		nextTlv->value[5] = (check>90) ? 1 : 0;	

		nextTlv->value[6]= 6;
		nextTlv->value[7] = (RF_CHCK()==BACK_SUCCESS) ? 1 : 0;	
		
		cnt= M[8];	
		check =EEPROM_Flush(MODE_ADDR,0xF0);	
		nextTlv->value[9] = (cnt<check) ? check : cnt;
		cnt = nextTlv->value[9];
		check =EEPROM_Flush(MODE_ADDR,0x0F);
		nextTlv->value[9] = (cnt<check) ? check : cnt;
		cnt = nextTlv->value[9];
		EEPROM_Flush(MODE_ADDR,0);
		nextTlv->value[9] = (cnt<check) ? check : cnt;
		cnt = nextTlv->value[9];
		EEPROM_B(M_ADDR+8,cnt);		
		nextTlv->value[8]= 7;	
		nextTlv->value[9] = (M[8]<2) ? 1 : 0;			
      	}
	else 
	{
	       nextTlv->type = 2; 		
	       nextTlv->len = 8;
	       for(check=0;check<8;check++)  nextTlv->value[7-check]= SSN[check];
	       
		Convert_Power(1); 
		nextTlv = getNextTlv(nextTlv);
		nextTlv->type = 7;
		nextTlv->len = 1;
		if(MODE[8]>=result_power) check=(u8)((MODE[9]+result_power)/2);		
		else check=result_power;
		nextTlv->value[0] = check;		


		//Delay_ms(20);	
	        Delay_ms(20);
		Convert_Temp(); 
		nextTlv = getNextTlv(nextTlv);
		nextTlv->type = 5;
		nextTlv->len = 2; 
		//value = Convert_BH1750();
		value = (s16)(result_temp * 100);
		value = ((value >> 8) & 0xff) | ((value << 8) &0xFF00);
		*(u16*)nextTlv->value = value;	
		Delay_ms(20);	
				
		Convert_Humi(); 
		nextTlv = getNextTlv(nextTlv);
		nextTlv->type = 6;
		nextTlv->len = 2;	
		value = (s16)(result_humi * 100);
		value = ((value >> 8) & 0xff) | ((value << 8) &0xFF00);
		*(s16*)nextTlv->value = value;
		
		nextTlv = getNextTlv(nextTlv);
		nextTlv->type = 9;
		nextTlv->len = 2; 
		light = Convert_BH1750();
		light =((light >> 8) & 0xff) | ((light << 8) &0xFF00);
		*(u16*)nextTlv->value = light;		
		
		if(SYS_Status!= 0)
	  	{
		      STORE_STAT();
		      nextTlv = getNextTlv(nextTlv);
		      nextTlv->type = 4;
		      nextTlv->len = 1;	
		      nextTlv->value[0] = SYS_Status;			      
		}
		if(COUT[0]==9)
		{	
		      nextTlv = getNextTlv(nextTlv);
		      nextTlv->type = 254;
		      nextTlv->len = 4;	
		      check=RRSSI[0]; 				
		      nextTlv->value[0]= RRSSI[check];
		      nextTlv->value[1]=STORE_TIME(0);
		      nextTlv->value[2]=COUT[0];
		      nextTlv->value[3]=COUT[8];
		}
	}		
	nextTlv = getNextTlv(nextTlv);
	nextTlv->type = 0;	
	
	computCheckSum(firstTlv,&check);
	firstTlv->value[0] = check;

	check =checkDatagramValid(firstTlv);		
	if(TLV_SUCCESS != check)  return BACK_ERROR; 
	return BACK_SUCCESS;
}
	 	
/*******************************************************************************
* Function Name  : RF_Send
* Description    : 
*******************************************************************************/
int RF_Send(u8 addr)
{	
      if(TxBuf[1]!=0 && TxBuf[2]!=0)
      {
	      TxBuf[0]=addr;	 
	      halRfSendPacket(TxBuf,sizeof(TxBuf));
	      STORE_TX(TxBuf);
	      STORE_TIME(FLAG_SEND);	
	      return BACK_SUCCESS;
      }
      return BACK_ERROR;
}


/*******************************************************************************
* Function Name  : RF_Rec
* Description    : 
*******************************************************************************/
u8 RF_Rec(u8 sec)
{	
	DATA_TLV_S* firstTlv;
	DATA_TLV_S* nextTlv;
	u8 check,cnt;	
	memset(RxBuf,0,sizeof(RxBuf));		
	RTC_Config(1000*sec);
	wfi();
	firstTlv = getFirstTlv(RxBuf+1);
	if(checkDatagramValid(firstTlv))
	{
	      STORE_TIME(FLAG_RECV);
	      STORE_RX(RxBuf);
	      FLAG_RSSI=GetRSSI();
	      STORE(RSSI_ADDR,15,FLAG_RSSI);	      
	      nextTlv = getNextTlv(firstTlv);
	      //if(RxBuf[4]==0XFA && RxBuf[3]==0xFA) { for(FLAG_TIME=90;FLAG_TIME>0;FLAG_TIME--){ LED_PORT->ODR ^= 0x08;Delay_ms(600);}}
	      while(nextTlv->type!=0)
	      {
		    if(nextTlv->type==253 && nextTlv->len==8 && FLAG_MODE==WORK_MODE) 
		    {
		          cnt=0;
			  for(check=0;check<8;check++){  if(nextTlv->value[8-check]==SSN[check]) cnt++;}				 							  
			  if(cnt==8)   Power_OFF();	
		    }
		    else if(nextTlv->type==8 && FLAG_MODE==PAIR_MODE)
		    {	
		           LED_PORT->ODR &= 0xF3;
			   //RF_TX();
			   check=MAIL[3];			   
			   STORE_MAIL(nextTlv->value);			   
			   Get_TLV();
			   Delay_ms(100);
			   LED_PORT->ODR |= 0x0C;
			   RF_Send(MAIL[3]);
			   Delay_ms(100);
			   LED_PORT->ODR &= 0xF3;
			   RF_Send(check);
			   Delay_ms(100);
			   LED_PORT->ODR |= 0x0C;			   			   
			   RF_Send(MAIL[3]);
			   Delay_ms(100);
			   LED_PORT->ODR &= 0xF3;
			   RF_Send(check);		   
			   Delay_ms(100);
			   LED_PORT->ODR |= 0x0C;
			   return BACK_SUCCESS;		
		     }
		     else if(nextTlv->type==3 && nextTlv->len==1 && FLAG_MODE==WORK_MODE)
		     {	
			   if(nextTlv->value[0]==0)    Reset();
			   else if(nextTlv->value[0]==1)   ;//Report(0x19);
			   return BACK_SUCCESS;
		     }
		     else if(nextTlv->type==0xFA)  
		     {		
		           
			   if(nextTlv->len==0) 
			   {	
			           //{ for(FLAG_TIME=90;FLAG_TIME>0;FLAG_TIME--){ LED_PORT->ODR ^= 0x08;Delay_ms(600);}}
			           MODE_Mark(CHCK_MODE);
				   LED_PORT->ODR &= 0xF3;
				   if(MODE[0]!=CHCK_MODE) { for(FLAG_TIME=100;FLAG_TIME>0;FLAG_TIME--){ LED_PORT->ODR ^= 0x04;Delay_ms(50);}}	
				   return BACK_SUCCESS;	
			   }
			   else if(nextTlv->len==8) 
			   {		
			     	   for(check=0;check<8;check++){  if(nextTlv->value[8-check]==SSN[check]) cnt++;}			 							  
				   if(cnt==8)  { MODE_Mark(0);Power_OFF();}
				   for(FLAG_TIME=200;FLAG_TIME>0;FLAG_TIME--){ LED_PORT->ODR ^= 0x04;Delay_ms(90);}
			   }
		     }
		     nextTlv = getNextTlv(nextTlv);
		}	     	     
	}
       return BACK_ERROR;
}

/*******************************************************************************
* Function Name  : STORE_MAIL
*******************************************************************************/
u8 STORE_MAIL(u8 *mail)
{
     u8 buff[16],ret;
     for(ret=0;ret<10;ret++)  buff[ret]=*(mail+ret);//EEPROM_B(MAIL_ADDR+ret,*(mail+ret));//
      buff[10]=MAIL[0];
      buff[11]=MAIL[1];
      buff[12]=MAIL[2];
      buff[13]=MAIL[3];
      buff[14]=MAIL[4]; 
      buff[15]=MAIL[15]+1; 
      ret = EEPROM_W(MAIL_ADDR,buff,16);
      return ret;
}
///*******************************************************************************
//* Function Name  : STORE_STAT
//*******************************************************************************/
void STORE_STAT(void)
{
    u8 buff=MODE[4];
    if(buff!=SYS_Status)  EEPROM_B(MODE_ADDR+4,SYS_Status);
    if(buff>0x0f)  SYS_Status|=(buff & 0xF0);
}
/*******************************************************************************
* Function Name  : STORE_STAT
*******************************************************************************/
u8 STORE(u16 addr,u8 num,u8 Data)
{
     u8 buff,cnt;  
     cnt=FLASH_ReadByte(addr);
     buff = FLASH_ReadByte(addr+cnt);
     if(Data!=buff && Data!=0)
     {
	  if(cnt>num) cnt=0;
	  cnt++;
	  EEPROM_B(RSSI_ADDR,cnt);
	  EEPROM_B(RSSI_ADDR+cnt,Data);
     }       
     return cnt;
}

/*******************************************************************************
* Function Name  : STORE_DATA 
*******************************************************************************/
u8 STORE_TX(u8 *data)
{
    	u8 cnt;
	u16 addr;
	cnt = FLASH_ReadByte(EEPROM_TX);
	addr=EEPROM_TX+cnt*28+1;  
	
	if(cnt <10) cnt++;
	else cnt=0;
	
	EEPROM_B(EEPROM_TX,cnt);	
	EEPROM_W(addr,data,28);             
        return cnt;	
}
/*******************************************************************************
* Function Name  : STORE_DATA 
*******************************************************************************/
u8 STORE_RX(u8 *data)
{
    	u8 cnt;
	u16 addr;
	cnt = FLASH_ReadByte(EEPROM_RX);	
	addr=EEPROM_RX+cnt*20+1; 
	
	if(cnt<12) cnt++;
	else cnt=0;
	
	EEPROM_B(EEPROM_RX,cnt);	
	EEPROM_W(addr,data,20);             
        return cnt;	
}
/*******************************************************************************
* Function Name  : STORE_DATA 
*******************************************************************************/
u8 STORE_TIME(u8 kind)
{
  	u8 cnt,buff;
	if(kind==FLAG_SEND)
	{
	    for(cnt=0;cnt<8;cnt++) 
	    { 
		  buff=COUT[cnt];
		  if(buff<0xff)
		  {
		     EEPROM_B(COUT_ADDR+cnt,(buff+1));
		     break;
		  }
		  else
		  {
		     EEPROM_B(COUT_ADDR+cnt,0);
		     EEPROM_B((COUT_ADDR+cnt+1),(buff+1));
		  }
	    }
	}
	else if(kind==FLAG_RECV)
	{
	    for(cnt=8;cnt<16;cnt++) 
	    { 
		  buff=COUT[cnt];
		  if(buff<0xff)
		  {
		    EEPROM_B(COUT_ADDR+cnt,(buff+1));
		    break;
		  }
		  else
		  {
		     EEPROM_B(COUT_ADDR+cnt,0);
		     EEPROM_B((COUT_ADDR+cnt+1),(buff+1));
		  }
	    }
	}
	else
	{ 
	    cnt=1;
	    while(COUT[cnt]!=0) cnt++;
	    buff=cnt;
	    cnt=9;
	    while(COUT[cnt]!=0) cnt++;
	    buff+=cnt;
	}
      return buff;	
}
/*******************************************************************************
* Function Name  :EEPROM_Flush
*******************************************************************************/
u8 EEPROM_Flush(u16 head,u8 val)
{
  	 u8 check;
	 u16 cout,EError;
	 EError=0;
	 FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard); 
	do {  
	      FLASH_Unlock(FLASH_MemType_Data);
	      if(check>100)
	      {
		SYS_Status|=0x10;		
		break;
	      }
	      check++;
         }while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET); 
	
	 for(cout=head;cout<EEPROM_END;cout++)   *(PointerAttr uint8_t*)cout =val;
	 
	 FLASH_WaitForLastOperation(FLASH_MemType_Data);
	 while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
	  {
	   	if(check>200)
		{
		     SYS_Status|=0x10;
		     break;
		}
	  	check++;  
	  }
	  FLASH_Lock(FLASH_MemType_Data);
	 
	 
	  for(cout=head;cout<EEPROM_END;cout++) 
	  {
		check = FLASH_ReadByte(cout);
		if(check!=val)  EError++;
	  }  	  
         if(EError!=0)
	 { 
	  	 SYS_Status|=0x10;
		 if(EError>0xFF) EError=0xFF;
		 if(EError!=M[8]) EEPROM_B(M_ADDR+8,EError);
	 }
	 return EError;
}
/*******************************************************************************
* Function Name  :FLUSH
*******************************************************************************/
u8 FLASH_Flush(u16 head,u8 val)
{
  	 u8 check;
	 u16 cout,FError;
	 FError=0;
	FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);     
        do {  
	  	FLASH_Unlock(FLASH_MemType_Program);
	  	if(check>100)
		{
		  SYS_Status|=0x20;		  
		  break;
		}
	  	check++;
      	    }while (FLASH_GetFlagStatus(FLASH_FLAG_PUL) == RESET);  
	 for(cout=head;cout<DATA_END;cout++)   *(PointerAttr uint8_t*)cout =val;
	 FLASH_WaitForLastOperation(FLASH_MemType_Program);
	 while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET)
	 {
	   	 if(check>200)
		{
		   SYS_Status|=0x20;
		   break;
		}
	  	check++;  
	 }
         FLASH_Lock(FLASH_MemType_Program);
	 for(cout=head;cout<DATA_END;cout++) 
	  {
		check = FLASH_ReadByte(cout);
		if(check!=val)  FError++;
	  }  	  
         if(FError!=0) 
	 {  
	   	 SYS_Status|=0x20;
		 if(FError>0xFF) FError=0xFF;
		 if(FError!=M[9]) EEPROM_B(M_ADDR+9,FError);
	 }
	return FError;
}



/*******************************************************************************
* Function Name  :RESET
*******************************************************************************/
void Reset(void)
{
        STORE_STAT();
	WWDG->CR = 0x80;
	WWDG->CR &= 0xBF;
}
/*******************************************************************************
* Function Name  : EEPROM_W
*******************************************************************************/

u8 EEPROM_W(u16 addr,u8 *data,u8 num)
{
      u8 check=0;
      u8 *eep;
      eep= (u8*)addr;
      FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
      do
      {  
	    FLASH_Unlock(FLASH_MemType_Data);
	    if(check==100)
	    {
	      SYS_Status|=0x10;
	      return BACK_TIMEOUT;
	    }
	    check++;
       }while((FLASH->IAPSR & FLASH_IAPSR_DUL) == 0);     
     while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
     while(num--)    *eep++ = *data++;
     FLASH_WaitForLastOperation(FLASH_MemType_Data);
     while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET);
     FLASH_Lock(FLASH_MemType_Data);
       return BACK_SUCCESS; 
}
/*******************************************************************************
* Function Name  : EEPROM_W
*******************************************************************************/

u8 EEPROM_B(u16 addr,u8 data)
{
      u8 check=0;
      FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);
      do
      {  
	  FLASH_Unlock(FLASH_MemType_Data);
	  if(check==100)
	  {
	    SYS_Status|=0x10;
	    return BACK_TIMEOUT;
	  }
	  check++;
       }while((FLASH->IAPSR & FLASH_IAPSR_DUL) == 0);
      
     while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET);
     *(PointerAttr uint8_t*)addr =data;  
     FLASH_WaitForLastOperation(FLASH_MemType_Data);
     while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET);
     FLASH_Lock(FLASH_MemType_Data);
     check = FLASH_ReadByte(addr);
     if(check==data) return BACK_SUCCESS;
     else
     {
       SYS_Status|=0x10;
       return BACK_ERROR; 
     }
}

/*******************************************************************************
* Function Name  : FLASH_W 
*******************************************************************************/
u8 FLASH_W(u16 addr,u8 *data,u8 num)
{
      u8 check=0;
      u16 abuff;
      abuff=addr;
      FLASH_SetProgrammingTime(FLASH_ProgramTime_Standard);     
      do {  
	  	FLASH_Unlock(FLASH_MemType_Program);
	  	if(check==100)
		{
		  SYS_Status|=0x20;
		  return BACK_TIMEOUT;
		}
	  	check++;
      	  }while (FLASH_GetFlagStatus(FLASH_FLAG_PUL) == RESET);         
      for(abuff=0;abuff<num;abuff++)   *(PointerAttr uint8_t*)(addr+abuff) =*(data+abuff);   
      FLASH_WaitForLastOperation(FLASH_MemType_Program);
      while (FLASH_GetFlagStatus(FLASH_FLAG_HVOFF) == RESET);
      FLASH_Lock(FLASH_MemType_Program);	
     check = FLASH_ReadByte(addr);
     if(check==*data)  return BACK_SUCCESS;
      else
     {
       SYS_Status|=0x20;
       return BACK_ERROR; 
     }   
}

