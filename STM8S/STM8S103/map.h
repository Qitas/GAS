
/*******************************************************************************
* Description    :STM8S103F3P3  TSSOP20 
*******************************************************************************/
#ifndef MAP_H__
#define MAP_H__
//#pragma once
#include <iostm8s103f3.h>
#include "type.h"
/*******************************************************************************
MODE[0],MODE[1]天阶段12min工作一次，MODE[2]日期
*******************************************************************************/
#define  			 HEAD_ADDR  	(uint16_t)0x4000 
__root __no_init volatile u8 	  MODE[16]  		@ 0x4000;

#define  			STORE_ADDR  	(uint16_t)0x4010 
__root __no_init volatile u8    DATA[112]  		@ 0x4010;

#define  			   LX_ADDR  	(uint16_t)0x4080 
__root __no_init volatile u8 	   LX[256]  		@ 0x4080;
#define  			   IN_ADDR  	(uint16_t)0x4180 
__root __no_init volatile u8 	   IN[128]  		@ 0x4180;
#define  			   EX_ADDR  	(uint16_t)0x4200 
__root __no_init volatile u8 	   EX[128]  		@ 0x4200;
#define  			 END_ADDR  	(uint16_t)0x427F 

/*******************************************************************************/

#define BACK_SUCCESS       0   /*成功*/
#define BACK_ERROR         1   /*错误*/
#define BACK_TIMEOUT       2   /*超时*/

#define   NIGHT         	0X33
#define   CLOUDY       		0X55  
#define   LAMP       		0X66 
#define   SUNNY         	0X88  

#define   LX_MAX      	    0X75
#define   LX_MIN      	    0X75
#define   UV_MAX      	    0X75
#define   UV_MIN      	    0X75
#define   HU_MAX      	    0X75
#define   HU_MIN      	    0X75
#define   TC_MAX      	    0X75
#define   TC_MIN      	    0X75
/*******************************************************************************
uv-adc2-pc4
humi of soil -adc3/4-up d2-down d3
temp/humi - io  
lx -iic
*******************************************************************************/


#define SPI_PORT      		 GPIOC
#define MISO_Pin      	    GPIO_Pin_7 
#define MOSI_Pin            GPIO_Pin_6 
#define SCLK_Pin            GPIO_Pin_5 
#define CS_Pin        	    GPIO_Pin_3  

#define ADC2_Pin            GPIO_Pin_4 
#define ADC_PORT   	         GPIOD
#define ADC4_Pin            GPIO_Pin_3 
#define ADC3_Pin            GPIO_Pin_2 
/*******************************************************************************/
void Delay_ms(uint16_t i);
void Delay_us(uint16_t i);
u8 EEPROM_B(u16 addr,u8 data);
void UASRT_SEND(u8 data) ;
#endif
