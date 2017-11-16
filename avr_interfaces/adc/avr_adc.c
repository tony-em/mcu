include "avr_adc.h"

void AVR_ADC_init(void)
{
  ADMUX |= (1 << REFS0) | (0 << REFS1); // select reference voltage
	ADMUX |= 0; // start 0 channel
	ADCSRA |= (1 << ADEN) | (0 << ADPS0) | (1 << ADPS1) | (1 < ADPS2); // enable ADC, set prescaler on 128
  //ADCSRA |= (1 << ADATE) | (1 << ADSC); SFIOR = 0x00; // enable infinite conversion - ???
}

void AVR_ADC_setChannel(uint8_t channel)
{
  ADMUX &= 0xE0;
  ADMUX |= channel;
}

uint32_t AVR_ADC_getValue(void)
{
  uint32_t value = 0;
	for (uint8_t i = 0; i < 3; i++) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		value += ADC;
	}

	return value / 3;
}
