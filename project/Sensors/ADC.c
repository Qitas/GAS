

/* Includes ------------------------------------------------------------------*/
#include "Console.h"
#include "stm8l15x_adc.h"
u8    result_power;
extern u8 SYS_Status;

u8 Convert_Power(u8 times)
{
    u16 adc1_value[6]; 
    u8 cnt;
    ADC_ChannelCmd(ADC1, ADC_Channel_Vrefint,ENABLE);
    ADC_Cmd(ADC1,ENABLE); 
    ADC_VrefintCmd(ENABLE);
    for(cnt=0;cnt<5;cnt++)
    {
	ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus (ADC1,ADC_FLAG_EOC));
	ADC_ClearFlag (ADC1,ADC_FLAG_EOC);
	adc1_value[cnt]=ADC_GetConversionValue(ADC1);
	Delay_ms(times);
    }
    //ADC_TempSensorCmd(DISABLE); 
    //ADC_DeInit(ADC1);
    ADC_ChannelCmd(ADC1, ADC_Channel_Vrefint,DISABLE);
    ADC_VrefintCmd(DISABLE);
    ADC_Cmd(ADC1 ,DISABLE);
    
    adc1_value[5]=(u16)((adc1_value[0]+adc1_value[1]+adc1_value[2]+adc1_value[3]+adc1_value[4])/5);
    adc1_value[0]=(u16)(adc1_value[times]*0.82)/10; //0.82=1v
    adc1_value[1]=(u16)(adc1_value[times]*1.63);    //1.63=2v
    result_power=(u8)(((4095-adc1_value[1])*10)/adc1_value[0]);  
    
    if(result_power>100  && result_power<140) 
    {       
	 if(result_power>0x70 && MODE[10]!=DEBUG_MODE)  EEPROM_B(MODE_ADDR+10,DEBUG_MODE);
	 if(M[0] < result_power)  EEPROM_B(M_ADDR,result_power);
	 result_power=100;
    }
    else if(result_power<20 || result_power>140 )
    {
         SYS_Status|=0x40;
	 if(M[1] > result_power)  EEPROM_B(M_ADDR+1,result_power);
         else if(M[1]!=20) EEPROM_B(M_ADDR+1,20);	 
	 if(MODE[7]!=FLAG_ERROR)  EEPROM_B(MODE_ADDR+7,FLAG_ERROR);	
    }
    else
    {
   	 if(MODE[7]!=FLAG_OK)  EEPROM_B(MODE_ADDR+7,FLAG_OK);
    }
    return BACK_SUCCESS; 
}
