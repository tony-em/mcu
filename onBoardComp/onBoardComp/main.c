#define F_CPU 16000000UL

#include "tftlcd_ili9481.h"
#include "avr_i2c.h"
#include "avr_adc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// Time counter
#define MCS_TIMER0_OVF 1024 // время до переполнения таймера0 в мкс
#define MCS_TIMER0_TICK 4 // время тика таймера0 в мкс
volatile uint32_t timer0_ovf_count = 0; // число переполнений таймера0

// ADC
#define VOLTAGE_REF 5// Опорное напряжение АЦП
#define VAL_STEP (VOLTAGE_REF / 1023.0) // Величина дискреты АЦП

// RTC DS3231
#define RTC_SEC		  0x00
#define RTC_MIN		  0x01
#define RTC_HOUR	  0x02
#define RTC_DAY_WEEK  0x03
#define RTC_DAY_MONTH 0x04
#define RTC_MONTH	  0x05
#define RTC_YEAR	  0x06
#define RTC_CMD_WR    0b11010000
#define RTC_CMD_RD    0b11010001
uint8_t sec = 0, min = 0, hour = 0, day = 0, month = 0, year = 0; // Время и дата

// External EEPROM AT24C32N
#define EEPROM_EXT_CMD_WR 0b10101110
#define EEPROM_EXT_CMD_RD 0b10101111
// Адреса ППЗУ для пробега
#define EEPROM_EXT_ADDR_ODO  0x0001
#define EEPROM_EXT_ADDR_TRIP 0x012C

// GUI
uint16_t backgr, padding = 2, posX = 0, posY = 0;

// Pins
// Пины для определения передачи
#define GEAR1_PIN PINC2
#define GEAR2_PIN PINC3
#define GEAR3_PIN PINC4
#define GEAR4_PIN PINC5
#define GEAR5_PIN PINC6
#define GEAR6_PIN PINC7
// Индикатор включенной передачи
#define GEAR1 !(PINC & (1 << GEAR1_PIN))
#define GEAR2 !(PINC & (1 << GEAR2_PIN))
#define GEAR3 !(PINC & (1 << GEAR3_PIN))
#define GEAR4 !(PINC & (1 << GEAR4_PIN))
#define GEAR5 !(PINC & (1 << GEAR5_PIN))
#define GEAR6 !(PINC & (1 << GEAR6_PIN))

// Пины для тахометра и спидометра
#define TAHO_PIN PIND2
#define SPEEDO_PIN PIND3
volatile uint32_t mcs_rpm = 0, mcs_sp = 0, per_rpm = 0, per_sp = 0; // Периоды между импульсами в мкс от спидометра и тахометра
uint16_t rpm = 0, speed = 0; volatile uint16_t meters1 = 0, meters10 = 0; // Обороты двигателя(об/мин), скорость (км/ч), счетчик метров и 10 метров
uint8_t fullRot = 0, odoAndTripChangeFlag = 0;
volatile uint32_t odo = 0, trip = 0, km = 0; // Одометр, пробег поездки, счетчик километров
uint16_t prevMeters10 = 0; uint32_t prevKm = 0;

// Число опрашиваемых каналов для АЦП
#define ADC_MAX_CHANNELS 5
// Назначение каналов АЦП
#define VOLTMETER PINA0
#define TEMPERATURE PINA1
#define TEMPERATURE_ENGINE PINA2
#define TPS PINA3
#define FUEL_LEVEL PINA4
// Коэффициент делителя для напряжения борт. сети
#define VOLTAGE_DIV 10
float voltage = 0; // напряжение бортовой сети (0 - 55В)
int8_t temperature = 0; // температура окружающей среды (-40 - 100 градусов Цельсия)
uint8_t temperatureEngine = 0; // температура двигателя (градусы Цельсия)
uint8_t tps = 0, prevTps = 0; // датчик положения дроссельной заслонки (%)
uint8_t fuelLevel = 0, fl = 0, startFuelLevel = 0, prevFuelLevel = 0, fuelFlag = 0; // уровень топлива (%)

// Поправочные коэффициенты
#define VOLTMETER_EPS (0.2)
#define TEMPERATURE_EPS (-5)

// Расход топлива(л/100км) и запас хода(км)
float fuelExpendL = 0; 
uint16_t reserveDistanceKM = 0;

// colors
uint16_t red, yellow, orange, green, blue;

// Тайминги обновления данных(мкс)
#define TIME1 1000000 // 1c
#define TIME2 250000 // 0.25c
#define TIME3 100000 // 0.1c
uint32_t time1 = 0, time2 = 0, time3 = 0, minute5 = 0;

// Настройки характеристик ТС
#define RPM_IMPULSE_ON_ROTATE 2 // число импульсов на один оборот двигателя
#define SPEED_IMPULSE_ON_ROTATE 4 // число импульсов на один оборот колеса
#define SPEED_FULL_RADIUS 50.18 // полный радиус колеса в см ([внутренний радиус + высота профиля] покрышки)
#define SPEED_DISTANCE_ON_ROTATE 185 // дистанция в см, которую проходит колесо за один оборот
#define FUEL_MAX_LEVEL 18 // емкость топливного бака в литрах


uint16_t calcRPM(uint32_t period_mcs);
void getRPM(void);
uint16_t calcSpeed(uint32_t period_mcs);
void getSpeed(void);
void setupGUI(void);
void setGUI_Rpm(void);
void setGUI_Speed(void);
void setGUI_TripAndOdo(void);
uint8_t getGear(void);
void setGUI_Gear(uint8_t gear);
void setGUI_Datetime(void);
void getDatetimeRTC_I2C(void);
void setDatetimeRTC_I2C(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint8_t year);
uint8_t binDecToDatetimeRTC(uint8_t t);
uint8_t datetimeToBinDecRTC(uint8_t t);
void setGUI_FuelLevelIndicator(void);
void setGUI_TpsIndicator(void);
void setGUI_Voltage(void);
void setGUI_TempEnviroment(void);
void setGUI_TempEngine(void);
void writeByteEEPROM_I2C(uint16_t addr, uint8_t dat);
uint8_t readByteEEPROM_I2C(uint16_t addr);
void writeBytesEEPROM_I2C(uint16_t addr, uint8_t *dats);
uint8_t* readBytesEEPROM_I2C(uint16_t addr);
void writeOdoAndTripEEPROM(void);
void readOdoAndTripEEPROM(void);
void setGUI_AvgFuelExpend(void);
void setGUI_AvgReserveDistance(void);
float calcAvgFuelExpend(uint8_t fuelLevel, uint8_t prevFuelLevel, uint32_t km, uint32_t meters10, uint8_t controlFlag);
uint16_t calcAvgReserveDistance(float fuelExpend, uint8_t fuelLevel);
void getAvgFuelExpend(void);
void getAvgReserveDistance(void);
uint32_t getMcs(void);
void getSensorsValues(void);

// Предустановки МК, инициализация аппаратуры и интерфейсов
void preset(void)
{
	TFTLCD_ILI9481_init();

	// Настройка пинов для отображения передачи двигателя
	DDRC = DDRC & 0b00000011;
	PORTC |= (1 << GEAR1_PIN) | (1 << GEAR2_PIN) | (1 << GEAR3_PIN) | (1 << GEAR4_PIN) | (1 << GEAR5_PIN) | (1 << GEAR6_PIN);

	// Настройка таймера T0 для счета мкс или мс
	// каждый тик таймера будет через 4 мкс
	// прерывание 8-битного таймера наступит через каждые 4*256=1024 мкс
	TCCR0 |= (1 << CS00) | (1 << CS01); // преддделитель частоты на 64
	TIMSK |= (1 << TOIE0); // режим работы таймера - прерывание по переполнению

	// Настройка аппаратных прерываний INT0(тахометр) и INT1(спидометр) по нарастающему фронту
	MCUCR |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11);
	GICR |= (1 << INT0) | (1 << INT1);
	
	// Инициализация шины I2C
	AVR_I2C_init();
	// Инициализвация ADC
	AVR_ADC_init();

	sei();
}

// Вывод значений датчиков на дисплей
void update(void)
{
	if (getMcs() >= time3 + TIME3) {
		time3 = getMcs();
		getDatetimeRTC_I2C();
		setGUI_Datetime();
	}
	
	if (getMcs() >= time2 + TIME2) {
		time2 = getMcs();
		getRPM();
		setGUI_Rpm();
		getSpeed();
		setGUI_Speed();
		setGUI_TripAndOdo();
		setGUI_Gear(getGear());
		
		getSensorsValues();
		setGUI_Voltage();
		setGUI_TempEnviroment();
		setGUI_TempEngine();
		setGUI_TpsIndicator();
	}
	
	if (getMcs() >= time1 + TIME1) {
		time1 = getMcs();
		setGUI_FuelLevelIndicator();
		minute5++;
	}

	if (minute5 >= 300) {
		minute5 = 0;
		getAvgFuelExpend();
		getAvgReserveDistance();
		setGUI_AvgFuelExpend();
		setGUI_AvgReserveDistance();
	}
}

// Инициализация начальных параметров и переменных
void init(void)
{
	red = TFTLCD_ILI9481_graph_get_color16(244,67,54);
	yellow = TFTLCD_ILI9481_graph_get_color16(255,235,59); 
	orange = TFTLCD_ILI9481_graph_get_color16(255,152,0);
	green = TFTLCD_ILI9481_graph_get_color16(0,150,136); 
	blue = TFTLCD_ILI9481_graph_get_color16(0,188,212);
	backgr = TFTLCD_ILI9481_graph_get_color16(25, 65, 165);

	readOdoAndTripEEPROM();
	getSensorsValues();

	time1 = getMcs(); time2 = time1; time3 = time1;
}

int main(void)
{
	preset();
	init();
	setupGUI();
	//setDatetimeRTC_I2C(0,20,5,26,12,17);
	while (1)
	{
		update();

		if (odoAndTripChangeFlag == 1) {
			writeOdoAndTripEEPROM();
		}
	}
}

// Функции получения времени работы МК в мкс и мс
uint32_t getMcs(void) 
{
	uint8_t oldSREG = SREG, t0;
	
	cli();
	t0 = TCNT0;
	SREG = oldSREG;

	return (timer0_ovf_count * MCS_TIMER0_OVF) + (t0 * MCS_TIMER0_TICK);
}

uint32_t getMs(void) 
{
	return getMcs() / 1000;
}

ISR(TIMER0_OVF_vect) 
{
	timer0_ovf_count++;
}

// Функции вывода графического пользовательского интерфейса и значений датчиков на дисплей
void setupGUI(void)
{
	TFTLCD_ILI9481_graph_fill_screen(backgr);
	// tps and rpm indicator
	TFTLCD_ILI9481_graph_draw_line(90, 319, 110, 265, WHITE);
	TFTLCD_ILI9481_graph_draw_line(110, 265, 285, 265, WHITE);
	TFTLCD_ILI9481_graph_draw_line(285, 265, 335, 229, WHITE);
	TFTLCD_ILI9481_graph_draw_line(335, 229, 479, 229, WHITE);
	// speed
	posX = 479 - padding - TFTLCD_ILI9481_getFontXpx()*4; posY = 229 - TFTLCD_ILI9481_getFontYpx();
	TFTLCD_ILI9481_graph_print_str(posX, posY, "KM/H", WHITE, backgr, 3);
	setGUI_Speed();
	// rpm
	TFTLCD_ILI9481_setFont(3);
	posX = 285 - padding - 3*TFTLCD_ILI9481_getFontXpx(); posY = 265 - TFTLCD_ILI9481_getFontYpx();
	TFTLCD_ILI9481_graph_print_str(posX, posY, "RPM", WHITE, backgr, 3);
	setGUI_Rpm();
	// gear
	TFTLCD_ILI9481_setFont(3);
	posX = padding + 7; posY = 319 - padding - TFTLCD_ILI9481_getFontYpx();
	TFTLCD_ILI9481_graph_print_str(posX, posY, "GEAR", backgr, WHITE, 3);
	setGUI_Gear(0);
	// fuel
	TFTLCD_ILI9481_setFont(4);
	posX = padding + 21 + 4*TFTLCD_ILI9481_getFontXpx(); posY = 65 + TFTLCD_ILI9481_getFontYpx();
	TFTLCD_ILI9481_graph_draw_line(0, posY, posX, posY, WHITE);
	TFTLCD_ILI9481_graph_print_str(padding + 10, 60, "FUEL", backgr, WHITE, 4);
	TFTLCD_ILI9481_graph_draw_line(0, posY + 105, posX, posY + 105, WHITE);
	TFTLCD_ILI9481_graph_draw_line(posX, posY - TFTLCD_ILI9481_getFontYpx() - 10, posX, posY + 105, WHITE);
	TFTLCD_ILI9481_graph_print_str(posX + padding, posY + 5, "H", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(posX + padding, posY + 108 - TFTLCD_ILI9481_getFontYpx() - 5, "L", WHITE, backgr, 4);
	// datetime
	TFTLCD_ILI9481_setFont(4);
	posX = 479 - padding - 8*TFTLCD_ILI9481_getFontXpx(); posY = 6*padding + TFTLCD_ILI9481_getFontYpx();
	setGUI_Datetime();
	// voltage
	TFTLCD_ILI9481_setFont(4);
	posX = 479 - 2*padding - TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_print_str(posX + 2, 2*padding + 16, "V", WHITE, backgr, 4);
	TFTLCD_ILI9481_setFont(6);
	TFTLCD_ILI9481_graph_print_str(posX - 51, 2*padding + 35, "VOLTAGE", backgr, WHITE, 6);
	setGUI_Voltage();
	// temp engine
	TFTLCD_ILI9481_setFont(4);
	posX = 479 - 3*padding - 6*TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_draw_vert_line(posX + 16, 0, 55, WHITE);
	TFTLCD_ILI9481_graph_print_str(posX - 2, 2*padding + 16, "C", WHITE, backgr, 4);
	posX = posX - TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_print_char(posX, 2*padding + 8, 96, WHITE, backgr, 4);
	TFTLCD_ILI9481_setFont(6);
	TFTLCD_ILI9481_graph_print_str(posX - 38, 2*padding + 35, "TEMP_ENG", backgr, WHITE, 6);
	setGUI_TempEngine();
	// temp env
	TFTLCD_ILI9481_setFont(4);
	posX = 316 - 2*padding - TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_draw_vert_line(posX + 16, 0, 55, WHITE);
	TFTLCD_ILI9481_graph_print_str(posX - 2, 2*padding + 16, "C", WHITE, backgr, 4);
	posX = 316 - 2*padding - 2*TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_print_char(posX, 2*padding + 8, 96, WHITE, backgr, 4);
	TFTLCD_ILI9481_setFont(6);
	TFTLCD_ILI9481_graph_print_str(posX - 18, 2*padding + 35, "TEMP", backgr, WHITE, 6);
	setGUI_TempEnviroment();

	// odo and trip
	TFTLCD_ILI9481_setFont(4);
	posX = 479 - padding - 13*TFTLCD_ILI9481_getFontXpx();
	TFTLCD_ILI9481_graph_print_str(posX, posY + TFTLCD_ILI9481_getFontXpx() + 16 + 2*padding, "TRIP", backgr, WHITE, 4);
	TFTLCD_ILI9481_graph_print_str(posX + 4*TFTLCD_ILI9481_getFontXpx(), posY + TFTLCD_ILI9481_getFontXpx() + 16 + 2*padding, ":", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(posX + 11*TFTLCD_ILI9481_getFontXpx(), posY + TFTLCD_ILI9481_getFontXpx() + 16 + 2*padding, "km", WHITE, backgr, 4);
	posX = 479 - padding - 12*TFTLCD_ILI9481_getFontXpx(); posY = posY + TFTLCD_ILI9481_getFontYpx()*2 + 16 + 8*padding;
	TFTLCD_ILI9481_graph_print_str(posX, posY, "ODO", backgr, WHITE, 4);
	TFTLCD_ILI9481_graph_print_str(posX + 3*TFTLCD_ILI9481_getFontXpx(), posY, ":", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(posX + 10*TFTLCD_ILI9481_getFontXpx(), posY, "km", WHITE, backgr, 4);
	setGUI_TripAndOdo();
	
	TFTLCD_ILI9481_graph_draw_horiz_line(0, 55, 479, WHITE);
	posY = posY + TFTLCD_ILI9481_getFontXpx() + 4*padding;
	TFTLCD_ILI9481_graph_draw_horiz_line(posX - 5, posY, 479-posX + 5, WHITE);
	TFTLCD_ILI9481_graph_draw_line(posX - 2*TFTLCD_ILI9481_getFontXpx() - 5, posY - 2*TFTLCD_ILI9481_getFontYpx() - 14*padding, posX - 5, posY, WHITE);
	TFTLCD_ILI9481_graph_draw_line(posX - 2*TFTLCD_ILI9481_getFontXpx() - 5, posY - 2*TFTLCD_ILI9481_getFontYpx() - 14*padding, posX - 2*TFTLCD_ILI9481_getFontXpx() - 5 - 16, posY - 2*TFTLCD_ILI9481_getFontYpx() - 14*padding - 28, WHITE);
	TFTLCD_ILI9481_graph_draw_line(posX - 2*TFTLCD_ILI9481_getFontXpx() - 5 - 16, posY - 2*TFTLCD_ILI9481_getFontYpx() - 14*padding - 28, posX - 2*TFTLCD_ILI9481_getFontXpx() - 8, 0, WHITE);

	TFTLCD_ILI9481_setFont(6);
	TFTLCD_ILI9481_graph_print_str(10 + padding, 18, "L/100km", WHITE, backgr, 6);
	setGUI_AvgFuelExpend();

	TFTLCD_ILI9481_setFont(6);
	TFTLCD_ILI9481_graph_print_str(15 + padding + 5*TFTLCD_ILI9481_getFontXpx(), 38, "km", WHITE, backgr, 6);
	setGUI_AvgReserveDistance();

	TFTLCD_ILI9481_graph_draw_horiz_line(0, 32, 15 + 2*padding + 3*TFTLCD_ILI9481_getFontXpx() + 6, WHITE);
	TFTLCD_ILI9481_graph_draw_line(15 + 2*padding + 3*TFTLCD_ILI9481_getFontXpx() + 6, 32, 15 + 2*padding + 3*TFTLCD_ILI9481_getFontXpx() + 20, 0, WHITE);
	TFTLCD_ILI9481_graph_draw_line(15 + 2*padding + 3*TFTLCD_ILI9481_getFontXpx() + 6, 32, 15 + 2*padding + 3*TFTLCD_ILI9481_getFontXpx() + 20, 55, WHITE);
}

void setGUI_Rpm(void)
{
	TFTLCD_ILI9481_setFont(2);
	TFTLCD_ILI9481_graph_print_int_format(rpm, 237 - 3*padding - TFTLCD_ILI9481_getFontXpx(), 265 - padding - TFTLCD_ILI9481_getFontYpx(), WHITE, backgr, 2, 5, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_Speed(void)
{
	TFTLCD_ILI9481_setFont(1);
	TFTLCD_ILI9481_graph_print_int_format(speed, 463 - 7*padding - 2*TFTLCD_ILI9481_getFontXpx(), 229 - padding - TFTLCD_ILI9481_getFontYpx(), WHITE, backgr, 1, 3, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_TripAndOdo(void)
{
	TFTLCD_ILI9481_setFont(4);
	TFTLCD_ILI9481_graph_print_int_format(trip, 479 - padding - 3*TFTLCD_ILI9481_getFontXpx(), 6*padding + 2*TFTLCD_ILI9481_getFontXpx() + 16 + 2*padding, WHITE, backgr, 4, 6, PRINT_DIR_RIGHT_TO_LEFT);
	TFTLCD_ILI9481_graph_print_int_format(odo, 479 - padding - 3*TFTLCD_ILI9481_getFontXpx(), 10*padding + 4*TFTLCD_ILI9481_getFontXpx() + 3*padding + 1, WHITE, backgr, 4, 6, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_Gear(uint8_t gear)
{
	TFTLCD_ILI9481_setFont(1);
	if (gear != 0) {
		TFTLCD_ILI9481_graph_print_int(gear, padding + 7 + 13, 319 - padding - 32 - 2*padding - TFTLCD_ILI9481_getFontYpx(), WHITE, backgr, 1);
	} else {
		TFTLCD_ILI9481_graph_print_char(padding + 7 + 13, 319 - padding - 32 - 2*padding - TFTLCD_ILI9481_getFontYpx(), 58, WHITE, backgr, 1);
	}
}

void setGUI_Datetime(void) 
{
	TFTLCD_ILI9481_setFont(4);
	uint16_t x = 224 - padding - 8*TFTLCD_ILI9481_getFontXpx();
	uint8_t h = datetimeToBinDecRTC(hour), m = datetimeToBinDecRTC(min), s = datetimeToBinDecRTC(sec),
	d = datetimeToBinDecRTC(day), mon = datetimeToBinDecRTC(month), y = datetimeToBinDecRTC(year);

	TFTLCD_ILI9481_graph_print_int(h >> 4, x, 3*padding, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(h & 0x0F, x + TFTLCD_ILI9481_getFontXpx(), 3*padding, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(x + 2*TFTLCD_ILI9481_getFontXpx(), 3*padding, ":", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(m >> 4, x + 3*TFTLCD_ILI9481_getFontXpx(), 3*padding, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(m & 0x0F, x + 4*TFTLCD_ILI9481_getFontXpx(), 3*padding, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(x + 5*TFTLCD_ILI9481_getFontXpx(), 3*padding, ":", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(s >> 4, x + 6*TFTLCD_ILI9481_getFontXpx(), 3*padding, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(s & 0x0F, x + 7*TFTLCD_ILI9481_getFontXpx(), 3*padding, WHITE, backgr, 4);

	uint8_t yy = 6*padding + TFTLCD_ILI9481_getFontYpx();
	TFTLCD_ILI9481_graph_print_int(d >> 4, x, yy, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(d & 0x0F, x + TFTLCD_ILI9481_getFontXpx(), yy, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(x + 2*TFTLCD_ILI9481_getFontXpx(), yy, "/", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(mon >> 4, x + 3*TFTLCD_ILI9481_getFontXpx(), yy, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(mon & 0x0F, x + 4*TFTLCD_ILI9481_getFontXpx(), yy, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_str(x + 5*TFTLCD_ILI9481_getFontXpx(), yy, "/", WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(y >> 4, x + 6*TFTLCD_ILI9481_getFontXpx(), yy, WHITE, backgr, 4);
	TFTLCD_ILI9481_graph_print_int(y & 0x0F, x + 7*TFTLCD_ILI9481_getFontXpx(), yy, WHITE, backgr, 4);
}

void setGUI_FuelLevelIndicator(void)
{
	if (fuelLevel < 25) {
		if (fl == 1) return;
		else fl = 1;
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 87, 2*padding + 77, 180, backgr);
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 156, 2*padding + 77, 180, red);
	} else if (fuelLevel < 50) {
		if (fl == 2) return;
		else fl = 2;
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 87, 2*padding + 77, 180, backgr);
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 133, 2*padding + 77, 180, orange);
	} else if (fuelLevel < 75) {
		if (fl == 3) return;
		else fl = 3;
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 87, 2*padding + 77, 180, backgr);
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 110, 2*padding + 77, 180, yellow);
	} else {
		if (fl == 4) return;
		else fl = 4;
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 87, 2*padding + 77, 180, backgr);
		TFTLCD_ILI9481_graph_fill_rect(padding + 2, 87, 2*padding + 77, 180, green);
	}
}

void setGUI_TpsIndicator(void)
{
	int8_t diff = tps - prevTps;
	uint8_t l;
	if (tps == 100)
	l = 20;
	else {
		l = (tps / 10)*2 + 1;
		if (tps % 10 > 5) l++;
	}
	
	if (diff == 0) return;
	else if (diff > 0) {
	uint16_t x = 115, color = WHITE;
	for (uint8_t i = 0; i < l; i++) {
		x+=13;
		if (i <= 9) {
			if (i >= 8) {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 270, x, 315, green);
			} else if (i >= 4) {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 270, x, 315, blue);
			} else {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 270, x, 315, color);
			}
		} else if (i == 10) {
			TFTLCD_ILI9481_graph_fill_rect(x-13, 263, x, 315, green);
		} else if (i == 11) {
			TFTLCD_ILI9481_graph_fill_rect(x-13, 250, x, 315, green);
		} else if (i == 12) {
			TFTLCD_ILI9481_graph_fill_rect(x-13, 237, x, 315, yellow);
		} else {
			if (i >= 17) {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 234, x, 315, red);
			} else if (i >= 15) {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 234, x, 315, orange);
			} else {
				TFTLCD_ILI9481_graph_fill_rect(x-13, 234, x, 315, yellow);
			}
		}
		x+=5;

		uint32_t t = getMcs();
		while(getMcs() < t + 10000);
	}
	} 
	else if (diff < 0) {
		uint16_t x = 488, color = backgr;
		for (uint8_t i = 20; i > l; i--) {
			x-=13;
			if (i <= 9) {
				TFTLCD_ILI9481_graph_fill_rect(x, 270, x+13, 315, color);
				} else if (i == 10) {
				TFTLCD_ILI9481_graph_fill_rect(x, 263, x+13, 315, color);
				} else if (i == 11) {
				TFTLCD_ILI9481_graph_fill_rect(x, 250, x+13, 315, color);
				} else if (i == 12) {
				TFTLCD_ILI9481_graph_fill_rect(x, 237, x+13, 315, color);
				} else {
				TFTLCD_ILI9481_graph_fill_rect(x, 234, x+13, 315, color);
			}
			x-=5;

			uint32_t t = getMcs();
			while(getMcs() < t + 10000);
		}
	}

	prevTps = tps;
}

void setGUI_Voltage(void)
{
	TFTLCD_ILI9481_setFont(5);
	TFTLCD_ILI9481_graph_print_float(voltage, 1, 480 - padding - 2*TFTLCD_ILI9481_getFontXpx(), 2*padding + 8, WHITE, backgr, 5, 4, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_TempEnviroment(void)
{
	TFTLCD_ILI9481_setFont(5);
	TFTLCD_ILI9481_graph_print_int_format(temperature, 312 - 4*padding - 2*TFTLCD_ILI9481_getFontXpx(), 2*padding + 8, WHITE, backgr, 5, 3, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_TempEngine(void)
{
	TFTLCD_ILI9481_setFont(5);
	TFTLCD_ILI9481_graph_print_int_format(temperatureEngine, 368 - TFTLCD_ILI9481_getFontXpx(), 2*padding + 8, WHITE, backgr, 5, 3, PRINT_DIR_RIGHT_TO_LEFT);
}

void setGUI_AvgFuelExpend(void)
{
	TFTLCD_ILI9481_setFont(4);
	if (fuelExpendL != 0) {
		TFTLCD_ILI9481_graph_print_float(fuelExpendL, 1, padding + 6, 2, WHITE, backgr, 4, 4, PRINT_DIR_LEFT_TO_RIGHT);
	} else {
		TFTLCD_ILI9481_graph_print_str_format(padding + 6, 2, "----", WHITE, backgr, 4, 4, PRINT_DIR_LEFT_TO_RIGHT);
	}
}

void setGUI_AvgReserveDistance(void)
{
	TFTLCD_ILI9481_setFont(4);
	if (reserveDistanceKM != 0) {
		TFTLCD_ILI9481_graph_print_int_format(reserveDistanceKM, padding + 6 + 2*TFTLCD_ILI9481_getFontXpx(), 36, WHITE, backgr, 4, 3, PRINT_DIR_RIGHT_TO_LEFT);
	} else {
		TFTLCD_ILI9481_graph_print_str_format(padding + 6 + 2*TFTLCD_ILI9481_getFontXpx(), 36, "---", WHITE, backgr, 4, 3, PRINT_DIR_RIGHT_TO_LEFT);
	}
}

// Рассчет среднего расхода топлива(л/100км) и среднего запаса хода(км)
void getAvgFuelExpend(void)
{
	float fuelExpendGlob = calcAvgFuelExpend(fuelLevel, startFuelLevel, km, meters10, 0);
	float fuelExpendLoc = calcAvgFuelExpend(fuelLevel, prevFuelLevel, abs(km - prevKm), abs(meters10 - prevMeters10), 1);
	
	if (fuelExpendLoc == -1 || fuelExpendGlob == -1) return;
	prevFuelLevel = fuelLevel;
	prevKm = km;
	prevMeters10 = meters10;

	if (fuelExpendLoc == 0) {
		if (fuelExpendGlob != 0) {
			if (fuelExpendL == 0) {
				fuelExpendL = fuelExpendGlob;
			} else {
				fuelExpendL = (fuelExpendL + fuelExpendGlob) / 2.0;
			}
		}
	} else {
		if (fuelExpendGlob != 0) {
			if (fuelExpendL == 0) {
				fuelExpendL = (fuelExpendGlob + fuelExpendLoc) / 2.0;
			} else {
				fuelExpendL = (fuelExpendL + fuelExpendGlob + fuelExpendLoc) / 3.0;
			}
		} else {
			fuelExpendL = (fuelExpendL + fuelExpendLoc) / 2.0;
		}
	}
}

void getAvgReserveDistance(void)
{
	uint16_t reserveDist = calcAvgReserveDistance(fuelExpendL, fuelLevel);
	if (reserveDist != 0) {
		if (reserveDistanceKM != 0) {
			reserveDistanceKM = (reserveDistanceKM + reserveDist) / 2;
		} else {
			reserveDistanceKM = reserveDist;
		}
	}
}

float calcAvgFuelExpend(uint8_t fuelLevel, uint8_t prevFuelLevel, uint32_t km, uint32_t meters10, uint8_t controlFlag)
{
	if (fuelLevel >= prevFuelLevel) return -1;
	float step = FUEL_MAX_LEVEL / 100.0;
	float diffFuel = (prevFuelLevel - fuelLevel)*step;
	if (diffFuel == 0) return 0;
	if (controlFlag != 0) {
		if (diffFuel >= step*7) return -1;
	}
	
	float expend = (diffFuel / ((float) km + (0.01*meters10)))*100;
	return (expend >= 20) ? 20 : expend;
}

uint16_t calcAvgReserveDistance(float fuelExpend, uint8_t fuelLevel)
{
	float fuel = (FUEL_MAX_LEVEL / 100.0)*fuelLevel;
	if (fuelExpend <= 1) return 0;
	else {
		float reserveDistance = (fuel / fuelExpend)*100;
		return (uint16_t) reserveDistance;
	}
}

// Расчет оборотов двигателя и скорости движения
uint16_t calcRPM(uint32_t period_mcs) {
	if (period_mcs == 0) return 0;
	else return (60000000 / (period_mcs * RPM_IMPULSE_ON_ROTATE));
}

void getRPM(void)
{
	uint16_t prevRpm = rpm, newRpm = (calcRPM(per_rpm) + calcRPM(per_rpm)) / 2;
	newRpm = (prevRpm + newRpm) / 2;
	if (prevRpm != 0) {
		uint8_t d = newRpm % 100;
		if (d <= 50) {
			rpm = newRpm - d;
		} else {
			rpm = newRpm - d + 100;
		}
		
	} else {
		rpm = newRpm;
	}
}

uint16_t calcSpeed(uint32_t period_mcs) {
	if (period_mcs == 0) return 0;
	else return ((36000 * SPEED_DISTANCE_ON_ROTATE) / (period_mcs * SPEED_IMPULSE_ON_ROTATE));
}

void getSpeed(void)
{
	uint16_t prevSp = speed, newSp = (calcSpeed(per_sp) + calcSpeed(per_sp) + calcSpeed(per_sp)) / 3;
	speed = (prevSp + newSp) / 2;
}

ISR(INT0_vect) {
	per_rpm = getMcs() - mcs_rpm;
	mcs_rpm = getMcs();
}

ISR(INT1_vect) {
	per_sp = getMcs() - mcs_sp;
	mcs_sp = getMcs();
	++fullRot;
	if (fullRot == SPEED_IMPULSE_ON_ROTATE) {
		fullRot = 0;
		meters1 += SPEED_DISTANCE_ON_ROTATE;
		if (meters1 >= 1000) {
			meters1 = meters1 % 1000;
			meters10 += 1;
		}
		if (meters10 >= 100) {
			meters10 = 0;
			++km;
			++trip;
			++odo;
			odoAndTripChangeFlag = 1;
		}
	}
}

// Функция для определения текущей передачи коробки передач
uint8_t getGear(void) {
	if (GEAR1) {
		return 1;
	} else if (GEAR2) {
		return 2;
	} else if (GEAR3) {
		return 3;
	} else if (GEAR4) {
		return 4;
	} else if (GEAR5) {
		return 5;
	} else if (GEAR6) {
		return 6;
	} else {
		return 0;
	}
}

// Значения с датчиков: напряжение бортсети, температура окр. среды и двигателя, уровня топлива, положения дроссельной заслонки
void getSensorsValues(void) {
	for (uint8_t p = 0; p < ADC_MAX_CHANNELS; p++) {
		AVR_ADC_setChannel(p);
		
		uint16_t val = (AVR_ADC_getValue() + AVR_ADC_getValue() + AVR_ADC_getValue()) / 3;
		switch (p) {
			case VOLTMETER:
				voltage = ((voltage + (val * VAL_STEP * (VOLTAGE_DIV + 1))+VOLTMETER_EPS) / 2);
				break;
			
			case TPS:
				tps = (tps + ((uint8_t)(0.0976 * val))) / 2;
				break;

			case TEMPERATURE:
				temperature = ((temperature + ((val * VAL_STEP * 100) - 273.15)+TEMPERATURE_EPS) / 2); // перевод из Кельвинов в градусы Цельсия
				break;

			case TEMPERATURE_ENGINE:
				temperatureEngine = (temperatureEngine + ((uint8_t)(0.127*val))) / 2;
				break;

			case FUEL_LEVEL:
				fuelLevel = (fuelLevel + ((uint8_t)(0.0976 * val))) / 2;
				if (fuelFlag == 0) {
					startFuelLevel = fuelLevel;
					prevFuelLevel = startFuelLevel;
					fuelFlag = 1;
				}
				break;
		}
	}
}

// Установка и получение текущего времени с RTC
void getDatetimeRTC_I2C(void)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(RTC_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(RTC_SEC);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_RESTART);
	AVR_I2C_setData(RTC_CMD_RD);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_RECEIVE_ACK);
	sec = binDecToDatetimeRTC(AVR_I2C_getData());
	AVR_I2C_action(I2C_RECEIVE_ACK);
	min = binDecToDatetimeRTC(AVR_I2C_getData());
	AVR_I2C_action(I2C_RECEIVE_ACK);
	hour = binDecToDatetimeRTC(AVR_I2C_getData());
	AVR_I2C_action(I2C_RECEIVE_ACK);
	AVR_I2C_action(I2C_RECEIVE_ACK);
	day = binDecToDatetimeRTC(AVR_I2C_getData());
	AVR_I2C_action(I2C_RECEIVE_ACK);
	month = binDecToDatetimeRTC(AVR_I2C_getData() & ~(1 << 7));
	AVR_I2C_action(I2C_RECEIVE_NACK);
	year = binDecToDatetimeRTC(AVR_I2C_getData());
	AVR_I2C_action(I2C_STOP);
}

void setDatetimeRTC_I2C(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint8_t year)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(RTC_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(RTC_SEC);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(sec));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(min));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(hour));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(0);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(day));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(month & ~(1 << 7)));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(datetimeToBinDecRTC(year));
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_STOP);
}

uint8_t datetimeToBinDecRTC(uint8_t t)
{
	return (t % 10) | (((t % 100) / 10) << 4);
}

uint8_t binDecToDatetimeRTC(uint8_t t)
{
	return ((t >> 4)*10) + (t & 0b00001111);
}

// Работа с внешней EEPROM
void writeByteEEPROM_I2C(uint16_t addr, uint8_t dat)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(EEPROM_EXT_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr >> 8);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(dat);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_STOP);
}

uint8_t readByteEEPROM_I2C(uint16_t addr)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(EEPROM_EXT_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr >> 8);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_RESTART);
	AVR_I2C_setData(EEPROM_EXT_CMD_RD);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_RECEIVE_NACK);
	uint8_t d = AVR_I2C_getData();
	AVR_I2C_action(I2C_STOP);
	return d;
}

void writeBytesEEPROM_I2C(uint16_t addr, uint8_t *dats)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(EEPROM_EXT_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr >> 8);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr);
	AVR_I2C_action(I2C_TRANSMIT);

	for (uint8_t i = 0; i < 4; i++) {
		AVR_I2C_setData(dats[i]);
		AVR_I2C_action(I2C_TRANSMIT);
	}

	AVR_I2C_action(I2C_STOP);
}

uint8_t* readBytesEEPROM_I2C(uint16_t addr)
{
	AVR_I2C_action(I2C_START);
	AVR_I2C_setData(EEPROM_EXT_CMD_WR);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr >> 8);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_setData(addr);
	AVR_I2C_action(I2C_TRANSMIT);
	AVR_I2C_action(I2C_RESTART);
	AVR_I2C_setData(EEPROM_EXT_CMD_RD);
	AVR_I2C_action(I2C_TRANSMIT);
	
	static uint8_t arr[4];
	for (uint8_t i = 0; i < 4 - 1; i++) {
		AVR_I2C_action(I2C_RECEIVE_ACK);
		arr[i] = AVR_I2C_getData();
	}
	AVR_I2C_action(I2C_RECEIVE_NACK);
	arr[4 - 1] = AVR_I2C_getData();
	
	AVR_I2C_action(I2C_STOP);
	return arr;
}

// Запись и чтение данных пробега в/из внешней EEPROM
void writeOdoAndTripEEPROM(void)
{
	uint8_t arr[4] = {odo >> 24, odo >> 16, odo >> 8, odo};
	writeBytesEEPROM_I2C(EEPROM_EXT_ADDR_ODO, arr);
	uint32_t t = getMcs();
	while(getMcs() < t + 11000);
	
	uint8_t a[4] = {trip >> 24, trip >> 16, trip >> 8, trip};
	writeBytesEEPROM_I2C(EEPROM_EXT_ADDR_TRIP, a);
	t = getMcs();
	while(getMcs() < t + 11000);
}

void readOdoAndTripEEPROM(void)
{
	uint8_t *arr = readBytesEEPROM_I2C(EEPROM_EXT_ADDR_ODO);
	odo = ((uint32_t)(*(arr++))) << 24;
	odo |= ((uint32_t)(*(arr++))) << 16;
	odo |= ((uint32_t)(*(arr++))) << 8;
	odo |= ((uint32_t)(*(arr++)));

	uint8_t *a = readBytesEEPROM_I2C(EEPROM_EXT_ADDR_TRIP);
	trip = ((uint32_t)(*(a++))) << 24;
	trip |= ((uint32_t)(*(a++))) << 16;
	trip |= ((uint32_t)(*(a++))) << 8;
	trip |= ((uint32_t)(*(a++)));
}