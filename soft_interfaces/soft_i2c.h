#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

#define SOFT_I2C_SDA PORTB2
#define SOFT_I2C_SCL PORTB3
#define SOFT_I2C_PORT PORTB
#define SOFT_I2C_DDR DDRB
#define SOFT_I2C_PIN PINB

#define SOFT_I2C_SDA_HIGH { SOFT_I2C_DDR &= ~(1 << SOFT_I2C_SDA); SOFT_I2C_PORT &= ~(1 << SOFT_I2C_SDA); }
#define SOFT_I2C_SDA_LOW { SOFT_I2C_DDR |= (1 << SOFT_I2C_SDA); SOFT_I2C_PORT &= ~(1 << SOFT_I2C_SDA); }
#define SOFT_I2C_SCL_HIGH { SOFT_I2C_DDR &= ~(1 << SOFT_I2C_SCL); SOFT_I2C_PORT &= ~(1 << SOFT_I2C_SCL); }
#define SOFT_I2C_SCL_LOW { SOFT_I2C_DDR |= (1 << SOFT_I2C_SCL); SOFT_I2C_PORT &= ~(1 << SOFT_I2C_SCL); }

#define SOFT_I2C_DELAY { _delay_us(50); }

#define SOFT_I2C_ACK 0
#define SOFT_I2C_NACK 1
