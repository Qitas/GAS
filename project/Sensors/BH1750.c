

#include "BH1750.h"
#include "i2c.h"

float result_lx=0;
u8 Data_BH1750[2]={0};

u16 LLX;


void Init_BH1750(void)
{
	i2c_Port_Init();
	Delay_ms(15);
}

void Cmd_Write_BH1750(u8 cmd)
{
    I2C_Start();                 
    I2C_Send_Byte(BH1750_Addr+0);  
    while(I2C_Wait_Ack());
    I2C_Send_Byte(cmd);    
    while(I2C_Wait_Ack());    
    I2C_Stop();              
    Delay_ms(5);
}

void Start_BH1750(void)
{
	Cmd_Write_BH1750(BH1750_ON);	
	Cmd_Write_BH1750(BH1750_RSET);	
	Cmd_Write_BH1750(BH1750_ONE);   
}
void Read_BH1750(void)
{ 
    i2c_Port_Init();
    Start_BH1750();
    Delay_ms(150);
    I2C_Start();                      
    I2C_Send_Byte(BH1750_Addr+1);       
    while(I2C_Wait_Ack());
    Data_BH1750[0]=I2C_Read_Byte(1);    
    Data_BH1750[1]=I2C_Read_Byte(0);     
    I2C_Stop();                
    Delay_ms(5);
}
u16 Convert_BH1750(void)
{
        Read_BH1750();
	LLX=Data_BH1750[0];
	LLX=(LLX<<8)+Data_BH1750[1];
	LLX=(u16)(LLX/1.2);	
	return LLX;
}


