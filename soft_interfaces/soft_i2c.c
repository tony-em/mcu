include "soft_i2c.h"

void initSoftI2C(void)
{
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;
}

void startSoftI2C(void) {
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SDA_LOW;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_LOW;
	SOFT_I2C_DELAY;
}

void restartSoftI2C(void) {
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;

	startSoftI2C();
}

void stopSoftI2C(void) {
	SOFT_I2C_SCL_LOW;
	SOFT_I2C_SDA_LOW;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_DELAY;
}

uint8_t getSoftI2CFlagACK(void) {
	uint8_t flag;
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;
	if (SOFT_I2C_PIN & (1 << SOFT_I2C_SDA)) {
		flag = 1;
	} else {
		flag = 0;
	}
	SOFT_I2C_SCL_LOW;
	SOFT_I2C_DELAY;
	return flag;
}

void sendSoftI2CFlagACK(uint8_t endFlag) {
	if (endFlag & 1) {
		SOFT_I2C_SDA_HIGH;
	} else {
		SOFT_I2C_SDA_LOW;
	}
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_HIGH;
	SOFT_I2C_DELAY;
	SOFT_I2C_SCL_LOW;
	SOFT_I2C_DELAY;
	SOFT_I2C_SDA_HIGH;
	SOFT_I2C_DELAY;
}

uint8_t sendByteSoftI2C(uint8_t data) {
	uint8_t count = 8;
	do {
		if (((data >> (count - 1)) & 1)) {
			SOFT_I2C_SDA_HIGH;
		} else {
			SOFT_I2C_SDA_LOW;
		}

		SOFT_I2C_DELAY;
		SOFT_I2C_SCL_HIGH;
		SOFT_I2C_DELAY;
		SOFT_I2C_SCL_LOW;
		SOFT_I2C_DELAY;
	} while (--count);
	return getSoftI2CFlagACK();
}

uint8_t readByteSoftI2C(uint8_t endFlag) {
	uint8_t count = 8, res = 0;
	do {
		SOFT_I2C_SCL_HIGH;
		SOFT_I2C_DELAY;
		if (SOFT_I2C_PIN & (1 << SOFT_I2C_SDA)) res |= 1;
		if (count != 1) res = res << 1;
		SOFT_I2C_SCL_LOW;
		SOFT_I2C_DELAY;
	} while (--count);
	sendSoftI2CFlagACK(endFlag);

	return res;
}
