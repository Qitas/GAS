#ifndef __ADC_H
#define __ADC_H
#include "type_def.h"

#define CONVERSIONMODE   1
#define CONVERSIONMODE_SINGLE  0
#define CONVERSIONMODE_CONTINUOUS  1

void ADC_conf();
uint16_t ADC_GetConversionValue(void);

#endif