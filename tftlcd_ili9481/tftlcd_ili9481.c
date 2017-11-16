#include "tftlcd_ili9481.h"

uint16_t x_size,
		 y_size;

uint8_t fontSizeX_px,
		fontSizeY_px,
		fontOffset,
		fontCharnums;

void TFTLCD_ILI9481_preset(void)
{
	DATA_DDR = 0xFF;
 	DATA_PORT = 0x00;

 	COMMAND_DDR_1 |= (1 << RST) | (1 << RS) | (1 << WR) | (1 << RD);
 	COMMAND_DDR_2 |= (1 << CS);

	fontSizeX_px = fontText1[0];
	fontSizeY_px = fontText1[1];
	fontOffset = fontText1[2];
	fontCharnums = fontText1[3];
}

void TFTLCD_ILI9481_reset(void)
{
	CS_IDLE;
	WR_IDLE;
	RD_IDLE;
	RST_ACTIVE;
	_delay_ms(2);
	RST_IDLE;
	CS_ACTIVE;
	TFTLCD_ILI9481_write_command(SOFT_RESET);
	for (uint8_t i = 0; i < 3; i++) WR_STROBE;
	CS_IDLE;
}

void TFTLCD_ILI9481_write_command(uint8_t cmd)
{
	CD_COMM;
	RD_IDLE;
	CS_ACTIVE;
	DATA_PORT = cmd;
	WR_STROBE;
	CS_IDLE;
}

void TFTLCD_ILI9481_write_data(uint8_t data)
{
	CD_DATA;
	RD_IDLE;
	CS_ACTIVE;
	DATA_PORT = data;
	WR_STROBE;
	CS_IDLE;
}

void TFTLCD_ILI9481_write8(uint8_t byte)
{
	DATA_PORT = byte;
	WR_STROBE;
}

void TFTLCD_ILI9481_reg_write32(uint8_t reg, uint32_t data)
{
	CS_ACTIVE;
	CD_COMM;
	TFTLCD_ILI9481_write8(reg);
	CD_DATA;
	_delay_us(1);
	TFTLCD_ILI9481_write8(data >> 24);
	_delay_us(1);
	TFTLCD_ILI9481_write8(data >> 16);
	_delay_us(1);
	TFTLCD_ILI9481_write8(data >> 8);
	_delay_us(1);
	TFTLCD_ILI9481_write8(data);
	CS_IDLE;
}

uint8_t TFTLCD_ILI9481_read8()
{
	uint8_t x;

	RD_ACTIVE;
	_delay_us(10);
	x = DATA_PIN;
	RD_IDLE;

	return x;
}

uint16_t TFTLCD_ILI9481_read_id()
{
	uint16_t id;

	CS_ACTIVE;
	CD_COMM;
	TFTLCD_ILI9481_write8(DEVICE_CODE_READ);
	WR_STROBE;

	setDataPortReadMode();
	CD_DATA;
	_delay_us(50);

	TFTLCD_ILI9481_read8();
	TFTLCD_ILI9481_read8();
	TFTLCD_ILI9481_read8();
	id = TFTLCD_ILI9481_read8();
	id <<= 8;
	id |= TFTLCD_ILI9481_read8();

	CS_IDLE;
	setDataPortWriteMode();
	_delay_us(150);

	return id;
}

uint16_t TFTLCD_ILI9481_init(void)
{
	TFTLCD_ILI9481_preset();
	TFTLCD_ILI9481_reset();
	_delay_ms(750);

	uint16_t id = TFTLCD_ILI9481_read_id();

	TFTLCD_ILI9481_reset();

	TFTLCD_ILI9481_write_command(EXIT_SLEEP_MODE);
	_delay_ms(50);

	TFTLCD_ILI9481_write_command(POWER_SETTING);
	TFTLCD_ILI9481_write_data(0x07);
	TFTLCD_ILI9481_write_data(0x42);
	TFTLCD_ILI9481_write_data(0x18);

	TFTLCD_ILI9481_write_command(VCOM_CONTROL);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x07);
	TFTLCD_ILI9481_write_data(0x10);

	TFTLCD_ILI9481_write_command(POWERSET_NORMMODE);
	TFTLCD_ILI9481_write_data(0x01);
	TFTLCD_ILI9481_write_data(0x02);

	TFTLCD_ILI9481_write_command(PANNEL_DRIVING_SETTING);
	TFTLCD_ILI9481_write_data(0x10);
	TFTLCD_ILI9481_write_data(0x3B);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x02);
	TFTLCD_ILI9481_write_data(0x11);

	TFTLCD_ILI9481_write_command(FRAME_RATE_AND_INVERSCONTROL);
	TFTLCD_ILI9481_write_data(0x03);

	TFTLCD_ILI9481_set_rotation(1);

	TFTLCD_ILI9481_write_command(SET_PIXEL_FORMAT);
	TFTLCD_ILI9481_write_data(0x55);

	TFTLCD_ILI9481_write_command(SET_COLUMN_ADDRESS);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x01);
	TFTLCD_ILI9481_write_data(0x3F);

	TFTLCD_ILI9481_write_command(SET_PAGE_ADDRESS);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x00);
	TFTLCD_ILI9481_write_data(0x01);
	TFTLCD_ILI9481_write_data(0xE0);
	_delay_ms(50);

	TFTLCD_ILI9481_write_command(SET_DISPLAY_ON);
	TFTLCD_ILI9481_write_command(WRITE_MEMORY_START);

	return id;
}

void TFTLCD_ILI9481_set_rotation(uint8_t rotation) {
	TFTLCD_ILI9481_write_command(SET_ADDRESS_MODE);
	switch (rotation) {
		case 0:
			TFTLCD_ILI9481_write_data(0b00001010); // 0 deg
			x_size = DISP_WIDTH;
			y_size = DISP_HEIGHT;
			break;
		case 1:
			TFTLCD_ILI9481_write_data(0b00101011); // 90 deg
			x_size = DISP_HEIGHT;
			y_size = DISP_WIDTH;
			break;
		case 2:
			TFTLCD_ILI9481_write_data(0b00001001); // 180 deg
			x_size = DISP_WIDTH;
			y_size = DISP_HEIGHT;
			break;
		case 3:
			TFTLCD_ILI9481_write_data(0b10101000); // 270 deg
			x_size = DISP_HEIGHT;
			y_size = DISP_WIDTH;
			break;
	}
}

// Graphical functions
void TFTLCD_ILI9481_graph_flood(uint16_t color, uint32_t len)
{
	uint16_t blocks;
	uint8_t  i, hi = color >> 8, lo = color;

	CS_ACTIVE;
	CD_COMM;
	TFTLCD_ILI9481_write8(WRITE_MEMORY_START);
	CD_DATA;
	TFTLCD_ILI9481_write8(hi);
	TFTLCD_ILI9481_write8(lo);
	len--;

	blocks = (uint16_t) (len / 64);
	if (hi == lo) {
		while (blocks--) {
			i = 16;
			do {
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE;
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE;
			} while(--i);
		}

		for (i = (uint8_t) len & 63; i--;) {
			WR_STROBE; WR_STROBE;
		}
	} else {
		while(blocks--) {
			i = 16;
			do {
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
			} while(--i);
		}
		for (i = (uint8_t) len & 63; i--; ) {
			TFTLCD_ILI9481_write8(hi);
			TFTLCD_ILI9481_write8(lo);
		}
	}

	CS_IDLE;
}

void TFTLCD_ILI9481_graph_fill_screen(uint16_t color)
{
	TFTLCD_ILI9481_graph_set_draw_window(0, 0, x_size - 1, y_size - 1);
	TFTLCD_ILI9481_graph_flood(color, (uint32_t) x_size * (uint32_t) y_size);
}

void TFTLCD_ILI9481_graph_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	TFTLCD_ILI9481_graph_set_draw_window(x1, y1, x2, y2);
	TFTLCD_ILI9481_graph_flood(color, (uint32_t) (x2 - x1 + 1) * (uint32_t) (y2 - y1 + 1));
}

void TFTLCD_ILI9481_graph_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
	if (x < 0 || y < 0 || x >= x_size || y >= y_size) return;

	TFTLCD_ILI9481_graph_set_draw_window(x, y, x, y);
 	CS_ACTIVE;
 	CD_COMM;
 	TFTLCD_ILI9481_write8(WRITE_MEMORY_START);
 	CD_DATA;
 	TFTLCD_ILI9481_write8(color >> 8);
 	TFTLCD_ILI9481_write8(color);
 	CS_IDLE;
}

void TFTLCD_ILI9481_graph_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}

	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			TFTLCD_ILI9481_graph_draw_pixel(y0, x0, color);
			} else {
			TFTLCD_ILI9481_graph_draw_pixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

void TFTLCD_ILI9481_graph_draw_horiz_line(int16_t x, int16_t y, int16_t len, uint16_t color)
{
	int16_t x2;

	if (len <= 0 || y < 0 || y >= y_size || x >= x_size ||
	(x2 = (x + len - 1)) <  0) return;

	if(x < 0) {
		len += x;
		x = 0;
	}

	if(x2 >= x_size) {
		x2 = x_size - 1;
		len  = x2 - x + 1;
	}

	TFTLCD_ILI9481_graph_set_draw_window(x, y, x2, y);
	TFTLCD_ILI9481_graph_flood(color, len);
	TFTLCD_ILI9481_graph_set_draw_window(0, 0, x_size - 1, y_size - 1);
}

void TFTLCD_ILI9481_graph_draw_vert_line(int16_t x, int16_t y, int16_t len, uint16_t color)
{
	int16_t y2;

	if ((len <= 0) || (x < 0 ) || ( x >= x_size) ||
	(y >= y_size) || ((y2 = (y+ len - 1)) < 0)) return;

	if (y < 0) {
		len += y;
		y = 0;
	}

	if (y2 >= y_size) {
		y2 = y_size - 1;
		len = y2 - y + 1;
	}

	TFTLCD_ILI9481_graph_set_draw_window(x, y, x, y2);
	TFTLCD_ILI9481_graph_flood(color, len);
	TFTLCD_ILI9481_graph_set_draw_window(0, 0, x_size - 1, y_size - 1);
}

void TFTLCD_ILI9481_graph_draw_circle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	TFTLCD_ILI9481_graph_draw_pixel(x0  , y0+r, color);
	TFTLCD_ILI9481_graph_draw_pixel(x0  , y0-r, color);
	TFTLCD_ILI9481_graph_draw_pixel(x0+r, y0  , color);
	TFTLCD_ILI9481_graph_draw_pixel(x0-r, y0  , color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		TFTLCD_ILI9481_graph_draw_pixel(x0 + x, y0 + y, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 - x, y0 + y, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 + x, y0 - y, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 - x, y0 - y, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 + y, y0 + x, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 - y, y0 + x, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 + y, y0 - x, color);
		TFTLCD_ILI9481_graph_draw_pixel(x0 - y, y0 - x, color);
	}
}

void TFTLCD_ILI9481_graph_draw_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	TFTLCD_ILI9481_graph_draw_horiz_line(x1, y1, x2 - x1, color);
	TFTLCD_ILI9481_graph_draw_horiz_line(x1, y2, x2 - x1, color);
	TFTLCD_ILI9481_graph_draw_vert_line(x1, y1, y2 - y1, color);
	TFTLCD_ILI9481_graph_draw_vert_line(x2, y1, y2 - y1, color);
}

void TFTLCD_ILI9481_graph_draw_triang(int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
	TFTLCD_ILI9481_graph_draw_line(x0, y0, x1, y1, color);
	TFTLCD_ILI9481_graph_draw_line(x1, y1, x2, y2, color);
	TFTLCD_ILI9481_graph_draw_line(x2, y2, x0, y0, color);
}

void TFTLCD_ILI9481_graph_set_draw_window(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	CS_ACTIVE;
	uint32_t t;

	t = x1;
	t <<= 16;
	t |= x2;
	TFTLCD_ILI9481_reg_write32(SET_COLUMN_ADDRESS, t);
	t = y1;
	t <<= 16;
	t |= y2;
	TFTLCD_ILI9481_reg_write32(SET_PAGE_ADDRESS, t);

	CS_IDLE;
}

uint16_t TFTLCD_ILI9481_graph_get_color16(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void TFTLCD_ILI9481_graph_print_char(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t typeFont)
{
	TFTLCD_ILI9481_setFont(typeFont);

	TFTLCD_ILI9481_graph_set_draw_window(x, y, x + fontSizeX_px - 1, y + fontSizeY_px - 1);
	CS_ACTIVE;
	CD_COMM;
	TFTLCD_ILI9481_write8(WRITE_MEMORY_START);
	CD_DATA;
	int temp = ((c - fontOffset) * ((fontSizeX_px / 8) * fontSizeY_px)) + 4,
		p = 0;
	for (int j = 0; j < ((fontSizeX_px / 8) * fontSizeY_px); j++) {

		unsigned char ch = pgm_read_byte(&fontText1[temp]);
		switch (typeFont) {
			case 1:
				ch = pgm_read_byte(&fontSpAndG[temp]);
				break;
			case 2:
				ch = pgm_read_byte(&fontRPM[temp]);
				break;
			case 3:
				ch = pgm_read_byte(&fontText1[temp]);
				break;
			case 4:
				ch = pgm_read_byte(&fontText2[temp]);
				break;
			case 5:
				ch = pgm_read_byte(&fontSens[temp]);
				break;
		}

		for (int i = 0; i < 8; i++) {
			if ((ch & (1 << (7 - i))) != 0) {
				uint8_t hi = color >> 8, lo = color;
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
			} else {
				uint8_t hi = bg >> 8, lo = bg;
				TFTLCD_ILI9481_write8(hi); TFTLCD_ILI9481_write8(lo);
			}
			p++;
		}
		if (p >= fontSizeX_px) p = 0;
		temp++;
	}
	CS_IDLE;
}

void TFTLCD_ILI9481_graph_print_str(int16_t x, int16_t y, char *str, uint16_t color, uint16_t bg, uint8_t typeFont)
{
	int len = strlen(str);

	for (int i = 0; i < len; i++) {
		TFTLCD_ILI9481_graph_print_char(x + (i * fontSizeX_px), y, *str++, color, bg, typeFont);
	}
}

uint16_t TFTLCD_ILI9481_graph_print_str_format(int16_t x, int16_t y, char *str, uint16_t color, uint16_t bg, uint8_t typeFont, uint16_t prevLen, uint8_t leftOrRight)
{
	int len = strlen(str);

	if (leftOrRight == 0) {
		for (int i = 0; i < len; i++) {
			TFTLCD_ILI9481_graph_print_char(x + (i * fontSizeX_px), y, *str++, color, bg, typeFont);
		}

		if (len != prevLen && prevLen != 0) {
		    uint16_t step = abs(prevLen - len);
			TFTLCD_ILI9481_graph_fill_rect(x + (len*fontSizeX_px), y, x + (len*fontSizeX_px) + (step*fontSizeX_px), y + fontSizeY_px, bg);
		}

	} else {
		 for (int i = 0; i < len; i++) *str++;
		 *str--;
		 for (int i = 0; i < len; i++) {
		 	TFTLCD_ILI9481_graph_print_char(x - (i * fontSizeX_px), y, *str--, color, bg, typeFont);
		 }

		 if (len != prevLen && prevLen != 0) {
			uint16_t step = abs(prevLen - len);
			TFTLCD_ILI9481_graph_fill_rect(x - ((len-1)*fontSizeX_px) - (step*fontSizeX_px), y, x - ((len-1)*fontSizeX_px), y + fontSizeY_px, bg);
		}
	}

	return len;
}

void TFTLCD_ILI9481_graph_print_int(int32_t num, int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t typeFont)
{
	int length = 0;
	char filler = ' ',
		 buf[25],
		 st[27];

	uint8_t neg = 0;
	int c = 0, f = 0;

	if (num == 0) {
		if (length != 0) {
			for (c = 0; c < (length - 1); c++)
				st[c] = filler;
			st[c] = 48;
			st[c+1] = 0;
		} else {
			st[0] = 48;
			st[1] = 0;
		}
	} else {
		if (num < 0) {
			neg = 1;
			num = -num;
		}

		while (num > 0) {
			buf[c]= 48 + (num % 10);
			c++;
			num = (num - (num % 10)) / 10;
		}
		buf[c] = 0;

		if (neg == 1) {
			st[0] = 45;
		}

		if (length > (c + neg)) {
			for (int i = 0; i < (length - c - neg); i++) {
				st[i + neg] = filler;
				f++;
			}
		}

		for (int i = 0; i < c; i++) {
			st[i + neg + f] = buf[c - i - 1];
		}
		st[c + neg + f] = 0;

	}

	TFTLCD_ILI9481_graph_print_str(x, y, st, color, bg, typeFont);
}

uint16_t TFTLCD_ILI9481_graph_print_int_format(int32_t num, int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t typeFont, uint16_t prevLen, uint8_t leftOrRight)
{
	int length = 0;
	char filler = ' ',
	buf[25],
	st[27];

	uint8_t neg = 0;
	int c = 0, f = 0;

	if (num == 0) {
		if (length != 0) {
			for (c = 0; c < (length - 1); c++)
			st[c] = filler;
			st[c] = 48;
			st[c+1] = 0;
			} else {
			st[0] = 48;
			st[1] = 0;
		}
		} else {
		if (num < 0) {
			neg = 1;
			num = -num;
		}

		while (num > 0) {
			buf[c]= 48 + (num % 10);
			c++;
			num = (num - (num % 10)) / 10;
		}
		buf[c] = 0;

		if (neg == 1) {
			st[0] = 45;
		}

		if (length > (c + neg)) {
			for (int i = 0; i < (length - c - neg); i++) {
				st[i + neg] = filler;
				f++;
			}
		}

		for (int i = 0; i < c; i++) {
			st[i + neg + f] = buf[c - i - 1];
		}
		st[c + neg + f] = 0;

	}

	return TFTLCD_ILI9481_graph_print_str_format(x, y, st, color, bg, typeFont, prevLen, leftOrRight);
}

uint16_t TFTLCD_ILI9481_graph_print_float(double num, uint8_t dec, int16_t x, int16_t y, uint16_t color, uint16_t bg, uint8_t typeFont, uint16_t prevLen, uint8_t leftOrRight)
{
	int length = 0;
	char filler = ' ', divider = '.';

	char st[27];
	uint8_t neg=0;

	if (dec<1)
	dec=1;
	else if (dec>5)
	dec=5;

	if (num<0)
	neg = 1;

	dtostrf(num, length, dec, st);
	if (divider != '.')
	{
		for (int i=0; i<sizeof(st); i++)
		if (st[i]=='.')
		st[i]=divider;
	}

	if (filler != ' ')
	{
		if (neg)
		{
			st[0]='-';
			for (int i=1; i<sizeof(st); i++)
			if ((st[i]==' ') || (st[i]=='-'))
			st[i]=filler;
		}
		else
		{
			for (int i=0; i<sizeof(st); i++)
			if (st[i]==' ')
			st[i]=filler;
		}
	}

	return TFTLCD_ILI9481_graph_print_str_format(x, y, st, color, bg, typeFont, prevLen, leftOrRight);
}

uint8_t TFTLCD_ILI9481_getCountCharsInNum(int32_t num) {
	uint8_t c = (num <= 0) ? 1 : 0;
	while (num) {
		num = num / 10;
		c++;
	}

	return c;
}

uint16_t TFTLCD_ILI9481_getFontXpx(void)
{
	return fontSizeX_px;
}

uint16_t TFTLCD_ILI9481_getFontYpx(void)
{
	return fontSizeY_px;
}

void TFTLCD_ILI9481_setFont(uint8_t typeFont)
{
	switch (typeFont) {
		case 1:
			fontSizeX_px = fontSpAndG[0];
			fontSizeY_px = fontSpAndG[1];
			fontOffset = fontSpAndG[2];
			fontCharnums = fontSpAndG[3];
			break;
		case 2:
			fontSizeX_px = fontRPM[0];
			fontSizeY_px = fontRPM[1];
			fontOffset = fontRPM[2];
			fontCharnums = fontRPM[3];
			break;
		case 3:
			fontSizeX_px = fontText1[0];
			fontSizeY_px = fontText1[1];
			fontOffset = fontText1[2];
			fontCharnums = fontText1[3];
			break;
		case 4:
			fontSizeX_px = fontText2[0];
			fontSizeY_px = fontText2[1];
			fontOffset = fontText2[2];
			fontCharnums = fontText2[3];
			break;
		case 5:
			fontSizeX_px = fontSens[0];
			fontSizeY_px = fontSens[1];
			fontOffset = fontSens[2];
			fontCharnums = fontSens[3];
			break;
	}
}

void TFTLCD_ILI9481_graph_test()
{
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
	for (int i = 0; i < 8; i++) {
		TFTLCD_ILI9481_graph_fill_screen(TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand()) % 255));
	}
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
 	for (int i = 0; i < 6; i++) {
		TFTLCD_ILI9481_graph_fill_rect(i + 5, i + 5, i + 50, i + 50, TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand())));
		 _delay_ms(500);
 	}
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
	for (int i = 0; i < 4096; i++) {
		TFTLCD_ILI9481_graph_draw_pixel(abs(rand()) % (x_size - 1), abs(rand()) % (y_size - 1), TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand()) % 255));
	}
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
	for (int i = 0; i < 1024; i++) {
		TFTLCD_ILI9481_graph_draw_line(abs(rand()) % (x_size - 1), abs(rand()) % (y_size - 1), abs(rand()) % (x_size - 1), abs(rand()) % (y_size - 1),
		TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand()) % 255));
	}
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
	for (int i = 0; i < 1024; i++) {
		TFTLCD_ILI9481_graph_draw_circle((abs(rand()) % (x_size - 21)) + 20, (abs(rand()) % (y_size - 21)) + 20, 20, TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand()) % 255));
	}
	TFTLCD_ILI9481_graph_fill_screen(BLACK);
	for (int i = 0; i < 1024; i++) {
		TFTLCD_ILI9481_graph_draw_rect(abs(rand()) % (x_size - 1), abs(rand()) % (y_size - 1), abs(rand()) % (x_size - 1), abs(rand()) % (y_size - 1),
		TFTLCD_ILI9481_graph_get_color16(abs(rand()) % 255, abs(rand()) % 255, abs(rand()) % 255));
		_delay_ms(1);
	}
}
