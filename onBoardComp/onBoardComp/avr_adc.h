#ifndef AVR_ADC_H_
#define AVR_ADC_H_

#include <avr/io.h>

void AVR_ADC_init(void);
void AVR_ADC_setChannel(uint8_t channel);
uint16_t AVR_ADC_getValue(void);

#endif