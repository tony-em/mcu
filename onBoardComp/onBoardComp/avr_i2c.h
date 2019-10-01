#ifndef AVR_I2C_H_
#define AVR_I2C_H_

#define F_CPU 16000000UL

#include <avr/io.h>

#define I2C_FREQ 100000UL
#define I2C_PRESC 1
#define I2C_BITRATE_REG_VALUE ((((F_CPU / I2C_FREQ) / I2C_PRESC) - 16 ) / 2)

#define I2C_START	0
#define I2C_RESTART 1
#define I2C_STOP 2
#define I2C_TRANSMIT 3
#define I2C_RECEIVE_ACK 4
#define I2C_RECEIVE_NACK 5

#define I2C_STATUS_START_OK	0x08
#define I2C_STATUS_RESTART_OK	0x10
#define I2C_STATUS_TX_ADDR_WR_OK_ACK_OK 0x18
#define I2C_STATUS_TX_ADDR_WR_OK_ACK_NOK 0x20
#define I2C_STATUS_TX_ADDR_RE_OK_ACK_OK 0x40
#define I2C_STATUS_TX_ADDR_RE_OK_ACK_NOK 0x48
#define I2C_STATUS_TX_DATA_OK_ACK_OK 0x28
#define I2C_STATUS_TX_DATA_OK_ACK_NOK	0x30
#define I2C_STATUS_RX_DATA_OK_ACK_OK 0x50
#define I2C_STATUS_RX_DATA_OK_ACK_NOK	0x58
#define I2C_STATUS_MCU_LOST_BUS	0x38

void AVR_I2C_init(void);
uint8_t AVR_I2C_action(uint8_t action);
void AVR_I2C_setData(uint8_t data);
uint8_t AVR_I2C_getData(void);

#endif