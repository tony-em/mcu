#include <avr/io.h>

void AVR_ADC_init(void);
void AVR_ADC_setChannel(uint8_t channel);
uint8_t AVR_ADC_getValue(void);
