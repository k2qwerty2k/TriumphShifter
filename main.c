// 8MHz or 16MHz

// _FACTOR = (F_CPU / 8000000)

// CTC1 = 1 PWM11=0 PWM10=1 Resolution=8bit TimerTOP=ICR1 Freq=31250 ICR1=(255 * _FACTOR)
// CTC1 = WGM12
// PWM11 = WGM11
// PWM10 = WGM10

// PB1 - OUT PWM
// ADC0 - input ADC
// D2 - CAL_PIN (to ground)
// D5 - LED // OC0B LED
// D6 - LED // OC0A LED
// D3 - LED // OC2B LED

#if F_CPU != 8000000 && F_CPU != 16000000
#error "UNSUPPORTED F_CPU SPEED! Support only 8MHz or 16MHz"
#endif

#if !defined(USE_SERIAL) 
#define USE_SERIAL 0
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

#if USE_SERIAL
#include "serial.h"
#endif

#if \
	!defined(__AVR_ATmega48__) && !defined(__AVR_ATmega48A__) && !defined(__AVR_ATmega48P__) && !defined(__AVR_ATmega48PA__) && !defined(__AVR_ATmega48PB__) && \
	!defined(__AVR_ATmega88__) && !defined(__AVR_ATmega88A__) && !defined(__AVR_ATmega88P__) && !defined(__AVR_ATmega88PA__) && !defined(__AVR_ATmega88PB__) && \
	!defined(__AVR_ATmega168__) && !defined(__AVR_ATmega168A__) && !defined(__AVR_ATmega168P__) && !defined(__AVR_ATmega168PA__) && !defined(__AVR_ATmega168PB__) && \
	!defined(__AVR_ATmega328__) && !defined(__AVR_ATmega328P__)
#error "Support only ATmega48(a,p,pa,pb), ATmega88(a,p,pa,pb), ATmega168(a,p,pa,pb) and ATmega328p(b)"
#endif

#define NEED_FACTOR (F_CPU / 8000000) // factor for F_CPU speed, 1 or 2

#define MIN_PWM 0x0F<<1 // ~0.5v
#define MAX_PWM 0xF0 // ~4.5v

#define EEPROM_OSCCAL ((uint32_t*)0) // on some AVR first 4 byte in EEPROM contain OSCCAL value
#define EEPROM_MIN_ADC ((uint8_t*)4) // from 4 byte + 2 byte contain MIN ADC value
#define EEPROM_CENTER_MIN_ADC ((uint8_t*)6) // from 6 byte + 2 byte contain CENTER ADC value
#define EEPROM_CENTER_MAX_ADC ((uint8_t*)8) // from 8 byte + 2 byte contain CENTER ADC value
#define EEPROM_MAX_ADC ((uint8_t*)10) // from 10 byte + 2 byte contain MAX ADC value

#define CAL_PIN_DDR DDRD
#define CAL_PIN_PIN PIND
#define CAL_PIN_PORT PORTD
#define CAL_PIN_NUM 2

uint8_t calStep; // calibration step

uint16_t adcCenterMin; // 0x0001-0x01FD
uint16_t adcCenterMax; // 0x0002-0x01FE
uint16_t adcMin; // 0x0000-(adcCenterMin-1)
uint16_t adcMax; // (adcCenterMax+1)-0x01FF

volatile uint16_t adcPrev;
volatile uint16_t adcCur;
volatile uint32_t adcArr;
volatile uint8_t adcIndex;
volatile uint16_t adcArrMin;
volatile uint16_t adcArrMax;

volatile uint16_t cnt31250; // counter for 1 second (31250 ticks per second on PWM)
volatile uint32_t seconds;

uint16_t pwmVal;

void readEEPROM(void) {
	adcMin = eeprom_read_byte(EEPROM_MIN_ADC);
	adcCenterMin = eeprom_read_byte(EEPROM_CENTER_MIN_ADC);
	adcCenterMax = eeprom_read_byte(EEPROM_CENTER_MAX_ADC);
	adcMax = eeprom_read_byte(EEPROM_MAX_ADC);
	if((adcCenterMin > 0x01FD) || (0x0001 > adcCenterMin)) {
		adcCenterMin = 0xFF;
	}
	if((adcCenterMax > 0x01FE) || (0x0002 > adcCenterMax)) {
		adcCenterMax = 0x0100;
	}
	if(adcCenterMin>adcCenterMax) {
		adcCenterMin = adcCenterMax - 1;
	}

	if(adcMin > adcCenterMin) {
		adcMin = adcCenterMin - 1;
	}

	if(adcCenterMax > adcMax) {
		adcMax = adcCenterMax + 1;
	}
}

void initADC(void) {
	adcIndex = 0;
	adcCur = ((adcCenterMax + adcCenterMin) >> 1); // center
	adcPrev = adcCur;
	adcArr = 0;
	adcArrMin = 0x01FF;
	adcArrMax = 0x0000;

	ADMUX = (0 << REFS1) | (0 << REFS0) | // AREF, internal VREF turned off
	        (1 << ADLAR) | // ADC Left Adjust Result ADCH(8) ADCL(2...)
	        (0 << MUX3) | (0 << MUX2) | (0 << MUX1) |
	        (0 << MUX0) | // ADC0 select
	        0x00;

	ADCSRA = (1 << ADEN) |  // ADC Enable
	         (0 << ADSC) |  // NO ADC Start Conversion
	         (1 << ADIF) |  // Reset ADC Interrupt Flag
	         (1 << ADIE) |  // Enable ADC interrupt
#if (F_CPU == 8000000)
			(0 << ADPS2) | (0 >> ADPS1) | (1 << ADPS0) | //  ADC Prescale 2 // 8MHz ~ 307692HZ ~ 307KHz
#elif (F_CPU == 16000000)
			 (1 << ADPS2) | (1 >> ADPS1) | (0 << ADPS0) | //  ADC Prescale 4 // 16MHz ~ 307692HZ ~ 307KHz
#endif
	         (1 << ADATE) | // ADC Auto Trigger Enable
	         0x00;
}

void initPWM(void) {

	cnt31250 = 0;
	seconds = 0;

	// OC1A on PB1
	DDRB |= (1 << PB1); // PB1 as output
	PORTB &= ~(1 << PB1); // disable pullup on PB1

	TCCR1A = 0;    // ?Reset
	TCCR1B = 0;    // Reset
	TCNT1 = 0;     // Reset
	TIMSK1 = 0;    // Reset

	TCCR1A = (1 << COM1A1) |
	         (0 << COM1A0) | // WGM13:10 = 1110 // mode 14: Toggle OC1A on Compare // TOP is ICR1
	                         // Match, OC1B disconnected (normal port
	                         // operation). For all other WGM1 settings, normal
	                         // port operation, OC1A/OC1B disconnected.
	         (0 << COM1B1) |
	         (0 << COM1B0) | // Normal port operation, OC1A/OC1B disconnected.
	         (1 << WGM11) | (0 << WGM10) | // Waveform Generation Mode
	         0x00;

	TCCR1B = (0 << ICNC1) |                // Input Capture Noise Canceler
	         (0 << ICES1) |                // Input Capture Edge Select
	         (1 << WGM13) | (1 << WGM12) | // Waveform Generation Mode
	         (0 << CS12) | (0 << CS11) | (1 << CS10) | // Prescale=001
	         0x00;

	TCNT1 = 0x00;

	// set TOP for TIMER1 PWM
	// set PWM value to center
#if (F_CPU == 8000000)
	ICR1 = 0x00FF; // 255
	pwmVal = 0x007F; // 127
#elif (F_CPU == 16000000)
	ICR1 = 0x01FF; // 511
	pwmVal = 0x00FF; // 255
#endif

	OCR1A = pwmVal;

	TIMSK1 |= (1 << TOIE1); // enable overflow interrupt
}

ISR(ADC_vect) {
	uint16_t adcVal;
	adcVal = 0x0000 | ((ADCL & 0b11000000) >> 7); // we need only one high bit (ADLAR=1)
	adcVal |= (ADCH << 1); // after read ADCH free ADC converter and starts new conversion automaticly
	adcArr += adcVal;
	if(adcArrMin > adcVal) {
		adcArrMin = adcVal;
	}
	if(adcVal > adcArrMax) {
		adcArrMax = adcVal;
	}
	adcIndex++;
	if(adcIndex > 9) {
		adcArr -= (adcArrMin - adcArrMax);
		// adcCur = adcArr / 8;
		adcCur = (adcArr >> 3); // division by 8
		adcArr = 0;
		adcIndex = 0;
		adcArrMin = 0x01FF;
		adcArrMax = 0x0000;
	}
}

ISR(TIMER1_OVF_vect) {
	// atomic operation
	OCR1A = pwmVal; // Compatible for mega328 where OCR1x updated on BOTTOM

	cnt31250++;
	if(cnt31250 > 31250) {
		cnt31250 = 0;
		seconds++;
	}
}


void initCal(void) {
	calStep = 0;
	CAL_PIN_DDR &= ~(1<<CAL_PIN_NUM); // cal pin as input
	CAL_PIN_PORT |= (1<<CAL_PIN_NUM); // cal pin pullup
}

uint8_t checkCalIsDown(void) {
	uint8_t i;
	if(!(CAL_PIN_PIN & (1<<CAL_PIN_NUM))) { // calibrate is press
		// check calibrate is press more than 50ms
		for(i = 0 ; i < 50 ; i++) {
			_delay_ms(1);
			if(CAL_PIN_PIN & (1<<CAL_PIN_NUM)) {
				// calibrate release
				return 0;
			}
		}
		// wait for release
		while(!(CAL_PIN_PIN & (1<<CAL_PIN_NUM))) {
			_delay_ms(10);
		}
		return 1;
	}
	return 0;
}


void setup(void) {
	cli();

	calStep = 0;

	DDRB = DDRC = DDRD = 0x00;
	PORTB = PORTC = PORTD = 0x00;

	initCal();

	_delay_ms(100);

	readEEPROM();

	initADC();

	if(checkCalIsDown()) {
		calStep = 1; // enter to calibrating
	}

	sei();

	ADCSRA |= (1<<ADSC)|(1<<ADIF); // Reset interrupt flag and start new conversion
}

void loop(void) {

	if(0 != calStep) {
		// ...
		return;
	}

}

int main(void) {
	setup();

	while (1) {
		loop();
	}

	
	return 0;
}
