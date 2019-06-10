
/* Includes ------------------------------------------------------------------*/

//#include "stm8l15x.h"
#include "CC1101.h"
#include "BH1750.h"

extern float	result_temp;
extern float 	result_humi;
extern u8  	result_power;
extern u8 	SYS_Status;

void USART1_Init(void) ;
void USART1_SendStr(u8 *Str);

int Power_ON(void);
void Power_OFF(void);
void Reset(void);
void Work(void);
u8 Convert_Power(u8 bat);
void RTC_Config(uint16_t time);
int Get_TLV(void);
int Report(u8 cofig);
int RF_Send(u8 addr);
u8 RF_Rec(u8 sec);

void Data_Deal(void);


void STORE_STAT(void);
u8 STORE_MAIL(u8 *mail);
u8 STORE(u16 addr,u8 num,u8 Data);
u8 STORE_TIME(u8 kind);
u8 STORE_RSSI(u8 Data);

u8 STORE_RX(u8 *data);
u8 STORE_TX(u8 *data);

u8 FLASH_W(u16 addr,u8 *data,u8 num);
u8 FLASH_B(u16 addr,u8 data);
void SN_code(u8 sn);
u8 EEPROM_Flush(u16 head,u8 val);
u8 FLASH_Flush(u16 head,u8 val);
void FLUSH(u8 SN);