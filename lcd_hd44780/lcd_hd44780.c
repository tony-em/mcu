#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

// Ports and pins for lcd
#define DATA_DDR	DDRB
#define DATA_PORT	PORTB
#define GATE_DDR	DDRC
#define GATE_PORT	PORTC
#define RS_PIN		PINC0
#define RW_PIN		PINC1
#define E_PIN		PINC2

#define DATA_INTERFACE_4_OR_8_BIT  0 // 0 -> 4 bit, 1 -> 8 bit

#define CLEAR_DISP	0x01
#define RETURN_CURSOR_HOME_DISP	0x02
#define ENTRY_MODE_DECRADDR_NOSHIFT_DISP 0x04
#define ENTRY_MODE_INCRADDR_NOSHIFT_DISP 0x06
#define ENTRY_MODE_SHIFTRIGHT_DISP	0x05
#define ENTRY_MODE_SHIFTLEFT_DISP	0x07
#define DISP_ON_NO_CURSORS	0x0C
#define DISP_ON_WITH_CURSOR_DOWN	0x0E
#define DISP_ON_WITH_CURSOR_DOWN_BLINK	0x0F
#define DISP_ON_WITH_CURSOR_FULL_BLINK	0x0D
#define DISP_OFF	0x08
#define CURSOR_SHIFT_LEFT	0x10
#define CURSOR_SHIFT_RIGHT 0x14
#define DISP_SHIFT_LEFT	0x18
#define DISP_SHIFT_RIGHT 0x1C
#define DATA_INTERFACE_DISP_4BIT_ONELINE_5x8	0x20
#define DATA_INTERFACE_DISP_4BIT_TWOLINE_5x8	0x28
#define DATA_INTERFACE_DISP_4BIT_ONELINE_5x11	0x24
#define DATA_INTERFACE_DISP_4BIT_TWOLINE_5x11	0x2C
#define DATA_INTERFACE_DISP_8BIT_ONELINE_5x8	0x30
#define DATA_INTERFACE_DISP_8BIT_TWOLINE_5x8	0x38
#define DATA_INTERFACE_DISP_8BIT_ONELINE_5x11	0x34
#define DATA_INTERFACE_DISP_8BIT_TWOLINE_5x11	0x3C
#define SET_AC_DDRAM	0x80
#define SET_AC_CGRAM	0x40

#define E_PIN_STROBE() { GATE_PORT |= (1 << E_PIN); _delay_us(5); GATE_PORT &= ~(1 << E_PIN); _delay_us(5); }
#define RS_PIN_CMD() { GATE_PORT &= ~(1 << RS_PIN); }
#define RS_PIN_DATA() { GATE_PORT |= (1 << RS_PIN); }
#define RW_PIN_WRITE() { GATE_PORT &= ~(1 << RW_PIN); }
#define RW_PIN_READ() { GATE_PORT |= (1 << RW_PIN); }

void LCD_HD44780_init();

void preset(void) {
	DATA_DDR = 0xFF;
	DATA_PORT = 0x00;
	GATE_DDR |= (1 << RS_PIN) | (1 << RW_PIN) | (1 << E_PIN);

	LCD_HD44780_init();
}

void LCD_HD44780_write8(unsigned char mByte) {
	if (DATA_INTERFACE_4_OR_8_BIT != 0 && DATA_INTERFACE_4_OR_8_BIT != 1) return;

	if (DATA_INTERFACE_4_OR_8_BIT == 0) {
		RW_PIN_WRITE();
		DATA_PORT = (mByte >> 4);
		_delay_us(5);
		E_PIN_STROBE();

		DATA_PORT = mByte;
		E_PIN_STROBE();
	}

	if (DATA_INTERFACE_4_OR_8_BIT == 1) {
		RW_PIN_WRITE();
		DATA_PORT = mByte;
		_delay_us(5);
		E_PIN_STROBE();
	}

	RW_PIN_READ();
	_delay_us(50);
}

void LCD_HD44780_write4(unsigned char mByte) {
	RW_PIN_WRITE();
	DATA_PORT = (mByte >> 4);
	_delay_us(5);
	E_PIN_STROBE();
	RW_PIN_READ();

	_delay_us(50);
}

void LCD_HD44780_init(void) {
	if (DATA_INTERFACE_4_OR_8_BIT != 0 && DATA_INTERFACE_4_OR_8_BIT != 1) return;

	_delay_ms(50);
	GATE_PORT &= ~(1 << RS_PIN);

	if (DATA_INTERFACE_4_OR_8_BIT == 0) {
		LCD_HD44780_write4(0x30);
		_delay_ms(2);
		LCD_HD44780_write4(0x20);
		_delay_ms(2);
	}

	if (DATA_INTERFACE_4_OR_8_BIT == 1) {
		LCD_HD44780_write8(0x30);
		_delay_ms(2);
		LCD_HD44780_write8(0x20);
		_delay_ms(2);
	}

	LCD_HD44780_write8(DATA_INTERFACE_DISP_4BIT_TWOLINE_5x8);
	_delay_ms(2);
	LCD_HD44780_write8(DISP_ON_NO_CURSORS);
	_delay_ms(2);
	LCD_HD44780_write8(CLEAR_DISP);
	_delay_ms(2);
	LCD_HD44780_write8(ENTRY_MODE_INCRADDR_NOSHIFT_DISP);
	_delay_ms(2);
}

void LCD_HD44780_cmd_write(unsigned char cmd) {
	RS_PIN_CMD();
	LCD_HD44780_write8(cmd);
}

void LCD_HD44780_data_write(unsigned char dat) {
	RS_PIN_DATA();
	LCD_HD44780_write8(dat);
}

void LCD_HD44780_clear_display(void) {
	LCD_HD44780_cmd_write(CLEAR_DISP);
	_delay_ms(2);
}

void LCD_HD44780_return_home_cursor_position(void) {
	LCD_HD44780_cmd_write(RETURN_CURSOR_HOME_DISP);
	_delay_ms(2);
}

void LCD_HD44780_set_cursor_position(unsigned char posX, unsigned char posY) {
	if (posX > 15 || posY > 1) return;
	LCD_HD44780_cmd_write(SET_AC_DDRAM | ((0x40 * posY) + posX));
	_delay_us(50);
}

void LCD_HD44780_write_buff_chars(char *buff, char len) {
	while(len--) {
		LCD_HD44780_data_write(*(buff++));
	}
}

int main(void)
{
	preset();

	char buff[] = { 0xA0, 0x41, 0xA7, 0xA0, 0x45, 0x43 };
	LCD_HD44780_write_buff_chars(buff, 6);
	LCD_HD44780_set_cursor_position(0, 1);

	char c = 0;
	while (1)
    {
		LCD_HD44780_data_write(c++);
		LCD_HD44780_set_cursor_position(0, 1);
		_delay_ms(1000);
    }
}
