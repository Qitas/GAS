/******************** (C) COPYRIGHT  ���iCreateǶ��ʽ���������� ***************************
 * �ļ���  ��adc.c
 * ����    ��AD���ú�����   
 * ʵ��ƽ̨��iCreate STM8������
 * �Ĵ����汾  ��V2.0.0
 * ����    ��ling_guansheng  QQ��779814207
 * ����    ��
 * �޸�ʱ�� ��2012-6-16

 * iCreate STM8������Ӳ������
   STM8��PF0��(Ҳ����ADC2��AIN10)�ӵ��ǹ�������������

****************************************************************************************/


#include "adc.h"
#include "uart.h"


/**************************************************************************
 * ��������ADC_conf
 * ����  ��ADCģ���ʼ��
 * ����  ����
 *
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ���� 
 *************************************************************************/
void ADC_conf()
{
   ADC_CR1 = (0<<4)|(1<<1)|(0<<0);    //ADCʱ������Ƶ��Ϊ16MHz �������÷�Ƶϵ��Ϊ2  ����ת��ģʽ �Ƚ�ֹADCת��       
   ADC_CR2 = (1<<3)|(0<<1);           //���������Ҷ���  ��ֹɨ��ģʽ
  
   ADC_CSR = (0<<5)|(0xa<<0);         //�����ⲿ���� ��ֹת�������ж� ����ת��ͨ��ΪAIN10
   ADC_TDRH = 4;                      //��ֹAIN10ʩ���ش���������  
   ADC_CR1 |= 1;                      //��һ��д1�Ǵӵ͹���ģʽ���� 
   ADC_CR1 |= 1;                      //����һλ��1��������ٴ�д1����ADCת��
}

/**************************************************************************
 * ��������Send_ADC_Value
 * ����  ��ADCת�������ʾ����
 * ����  ��AD_Value--ADCת�����ֵ
 *
 * ���  ����
 * ����  ���� 
 * ����  ���ڲ����� 
 *************************************************************************/
static void Send_ADC_Value(u16 AD_Value)
{
    UART1_SendByte(AD_Value/1000+0x30);
    UART1_SendByte(AD_Value%1000/100+0x30);
    UART1_SendByte(AD_Value%1000%100/10+0x30);
    UART1_SendByte(AD_Value%1000%100%10+0x30);
}

/**************************************************************************
 * ��������ADC_GetConversionValue
 * ����  ����ȡADCת�����
 * ����  ����
 *
 * ���  ����
 * ����  ���� 
 * ����  ���ڲ����� 
 *************************************************************************/
uint16_t ADC_GetConversionValue(void)
{
  uint16_t value,temph;        
  uint8_t templ;                  // ����templ�洢��8λ����  temph�洢��8λ����
  
  while(!(ADC_CSR & 0x80));           //�ȴ�ת�����
  templ = ADC_DRL;
  temph = ADC_DRH;                  //��ȡADCת��  ���������Ҷ���ģʽ�� ��ȡ���ݵ�˳��ͬ  �ο�STM8�Ĵ���.PDFP371          
  
  value = (unsigned int)(templ | (temph << 8));   //ע����10λ��ת������ value��temphӦΪunsigned int ����
  return  value;
}