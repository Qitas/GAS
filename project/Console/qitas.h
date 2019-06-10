
/*******************************************************************************
            YuenJee wirless sensor for STM8
*******************************************************************************/
#ifndef YUENJEE_H__
#define YUENJEE_H__

#include "stm8l15x.h"



#define	  FLASH_HEAD  	 (uint16_t)0x8080
#define	  FLASH_END  	 (uint16_t)0xC000 

#define   			DATA_HEAD  	(uint16_t)0x8080
__root __no_init volatile u8 	DATA[1024]  		@ 0x8080;
#define   			 DATA_END  	(uint16_t)0x8480

#define	   			  SSN_ADDR  	(uint16_t)0xBFF0
__root __no_init volatile u8  	   SSN[17]  		@ 0xBFF0;

/*******************************************************************************
MODE[0-3] ����ģʽ���      MODE[4-9]�쳣���     MODE[10]����ģʽ��� 
*******************************************************************************/

#define  			 MODE_ADDR  	(uint16_t)0x1000 
__root __no_init volatile u8 	  MODE[16]  		@ 0x1000;
#define   			MAIL_ADDR  	(uint16_t)0x1010
__root __no_init volatile u8 	  MAIL[16]  		@ 0x1010;
#define  			 COUT_ADDR 	(uint16_t)0x1020
__root __no_init volatile u8 	  COUT[16]  	   	@ 0x1020;
#define  			 RSSI_ADDR 	(uint16_t)0x1030
__root __no_init volatile u8 	 RRSSI[16]  	   	@ 0x1030;

/*******************************************************************************
M[0]��ߵ�ѹ���     M[1]��͵�ѹ���      M[2]ǰ��ѹ���    M[3]���ѹ��� 
M[4]HSIʱ���ȶ�����  M[5]LSIʱ���ȶ�����  M[6]�����жϼ���
M[8]EEPROM          M[9]FLASH
M[12]�¶�ƫ��HSB    M[13]�¶�ƫ��LSB 
M[14]ʪ��ƫ��HSB    M[15]ʪ��ƫ��LSB
*******************************************************************************/
#define  			   M_ADDR  	(uint16_t)0x1040 
__root __no_init volatile u8 	     M[48]  		@ 0x1040;

#define  			TEMP_ADDR 	(uint16_t)0x1080
__root __no_init volatile u8 	  TEMP[64]  	   	@ 0x1080;
#define  			HUMI_ADDR 	(uint16_t)0x10C0
__root __no_init volatile u8 	  HUMI[64]  	   	@ 0x10C0;


#define  			POWR_ADDR 	(uint16_t)0x1100
__root __no_init volatile u8 	 POWR[256]  	   	@ 0x1100;

#define  			EEPROM_RX 	(uint16_t)0x1200
__root __no_init volatile u8 	EE_RX[256]  	   	@ 0x1200;

#define  			EEPROM_TX 	(uint16_t)0x1300
__root __no_init volatile u8 	EE_TX[256]  	   	@ 0x1300;

#define   EEPROM_END  	 (uint16_t)0x13FF 

#define 	BACK_SUCCESS       0   /*�ɹ�*/
#define 	BACK_ERROR         1   /*����*/
#define 	BACK_TIMEOUT       2   /*��ʱ*/

#define 	FLAG_SEND        0XF0   /*���*/
#define 	FLAG_RECV        0X0F   /*���*/

#define 	FLAG_OK          0x80   /*���*/
#define 	FLAG_ERROR       0x7F   /*���*/

#define  	PAIR_MODE 	0x88
#define  	WORK_MODE 	0x44
#define  	CHCK_MODE	0x22
#define  	DEBUG_MODE	0x11

#define 	SPI_PORT      GPIOB
#define 	MISO_Pin      GPIO_Pin_7 
#define 	MOSI_Pin      GPIO_Pin_6 
#define 	SCLK_Pin      GPIO_Pin_5 
#define 	CS_Pin        GPIO_Pin_2  

#define 	GDO_PORT      GPIOD
#define 	GDO0_Pin      GPIO_Pin_4 

#define 	IIC_PORT      	GPIOC
#define		SCL_Pin 	GPIO_Pin_1
#define		SDA_Pin   	GPIO_Pin_0

#define 	KEY_PORT   	GPIOB
#define 	KEY_Pin    	GPIO_Pin_4  	 
 
#define 	LED_PORT   		GPIOA
#define 	Blue_LED_Pin       GPIO_Pin_6	 	
#define 	Green_LED_Pin      GPIO_Pin_6 	 	

#define 	POWER_PORT   	        GPIOD
#define 	POWER_Pin          GPIO_Pin_5 

#define 	ADC_PORT   	        GPIOA
#define 	ADC_Pin            GPIO_Pin_4 

//-------------------------------------------------------------------------------------------------------
typedef unsigned char       BOOL;
typedef unsigned char       BYTE;
//-------------------------------------------------------------------------------------------------------


void Delay_ms(uint16_t i);
void Delay_us(uint16_t i);
u8 EEPROM_B(u16 addr,u8 data);
u8 EEPROM_W(u16 addr,u8 *data,u8 num);


#endif
