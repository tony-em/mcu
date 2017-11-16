#include "avr_i2c.h"

void AVR_I2C_init(void)
{
  TWBR = (uint8_t) I2C_BITRATE_REG_VALUE;
}

uint8_t AVR_I2C_action(uint8_t action)
{
  switch(action) {
		case I2C_START:
		case I2C_RESTART:
			TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // TWIE - enable interrupt;
			break;
		case I2C_STOP:
			TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
			break;
		case I2C_TRANSMIT:
			TWCR = (1 << TWEN) | (1 << TWINT);
			break;
		case I2C_RECEIVE_ACK:
			TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
			break;
		case I2C_RECEIVE_NACK:
			TWCR = (1 << TWEN) | (1 << TWINT);
			break;
	}

  if (action != I2C_STOP) while (!(TWCR & (1 << TWINT)));
  return (TWSR & 0xF8);
}

void AVR_I2C_data(uint8_t data)
{
  TWDR = data;
}
