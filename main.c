// 8MHz or 16MHz

// _FACTOR = (F_CPU / 8000000)

// CTC1 = 1 PWM11=0 PWM10=1 Resolution=8bit TimerTOP=ICR1 Freq=31250
// 	8MHz ICR1=255
// 	16MHz ICR1=511
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
#include <avr/pgmspace.h>

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

#if (F_CPU == 8000000)
#define PWM_MIN 0x0F // ~0.5v
#define PWM_CENTER 0x7F // ~2.5v
#define PWM_MAX 0xF0 // ~4.5v
#elif (F_CPU == 16000000)
#define PWM_MIN (0x0F<<1) // ~0.5v
#define PWM_CENTER (0x7F<<1) // ~2.5v
#define PWM_MAX (0xF0<<1) // ~4.5v
#endif

#define EEPROM_OSCCAL ((uint32_t*)0) // on some AVR first 4 byte in EEPROM contain OSCCAL value
#define EEPROM_CENTER_MIN_ADC ((uint16_t*)4) // from 6 byte + 2 byte contain CENTER ADC value
#define EEPROM_CENTER_MAX_ADC ((uint16_t*)6) // from 8 byte + 2 byte contain CENTER ADC value
#define EEPROM_MIN_ADC ((uint16_t*)8) // from 4 byte + 2 byte contain MIN ADC value
#define EEPROM_MAX_ADC ((uint16_t*)10) // from 10 byte + 2 byte contain MAX ADC value

#define CAL_PIN_DDR DDRD
#define CAL_PIN_PIN PIND
#define CAL_PIN_PORT PORTD
#define CAL_PIN_NUM 2

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_BLUE_PIN_NUM 3
#define LED_GREEN_PIN_NUM 5
#define LED_RED_PIN_NUM 6

#define LED_BLUE_OFF() (LED_PORT &= ~(1<<LED_BLUE_PIN_NUM))
#define LED_BLUE_ON() (LED_PORT |= (1<<LED_BLUE_PIN_NUM))
#define LED_BLUE_TOGGLE() (LED_PORT ^= (1<<LED_BLUE_PIN_NUM))

#define LED_GREEN_OFF() (LED_PORT &= ~(1<<LED_GREEN_PIN_NUM))
#define LED_GREEN_ON() (LED_PORT |= (1<<LED_GREEN_PIN_NUM))
#define LED_GREEN_TOGGLE() (LED_PORT ^= (1<<LED_GREEN_PIN_NUM))

#define LED_RED_OFF() (LED_PORT &= ~(1<<LED_RED_PIN_NUM))
#define LED_RED_ON() (LED_PORT |= (1<<LED_RED_PIN_NUM))
#define LED_RED_TOGGLE() (LED_PORT ^= (1<<LED_RED_PIN_NUM))


uint8_t calStep; // calibration step

// adc input after calibrating
// /not used/adcMin...adcCenterMin/not used/adcCenterMax...adcMax/not used/
uint16_t adcCenterMin; // 0x0001-0x01FD
uint16_t adcCenterMax; // 0x0002-0x01FE
uint16_t adcMin; // 0x0000-(adcCenterMin-1)
uint16_t adcMax; // (adcCenterMax+1)-0x01FF

volatile uint16_t adcPrev; // previous avg ADC value
volatile uint16_t adcCur; // current avg ADC value
volatile uint32_t adcArr; // last 10 ADC measurents
volatile uint8_t adcIndex; // ADC measure index
volatile uint16_t adcArrMin; // minimum ADC value in last 10 measurements
volatile uint16_t adcArrMax; // maximum ADC value in last 10 measurements

volatile uint16_t cnt31250; // counter for 1 second (31250 ticks per second on PWM)
volatile uint32_t seconds; // for blink leds and some other in future

// pwm output
// /not used/PWM_MIN...PWM_CENTER...PWM_MAX/not used/
volatile uint16_t pwmVal; // need PWM value to set
float pwmCenterToMaxPerStep = 0;
float pwmCenterToMinPerStep = 0;

void reCheckCenterAdcValues(void) {
	if((adcCenterMin > 0x01FD) || (0x0001 > adcCenterMin)) {
		adcCenterMin = 0xFF;
	}
	if((adcCenterMax > 0x01FE) || (0x0002 > adcCenterMax)) {
		adcCenterMax = 0x0100;
	}
	if(adcCenterMin > adcCenterMax) {
		adcCenterMin = adcCenterMax - 1;
	}
}

void reCheckAdcValues(void) {
	if(adcMin > adcCenterMin) {
		adcMin = adcCenterMin - 1;
	}

	if((adcCenterMax > adcMax) || (adcMax > 0x01FF)) {
		adcMax = adcCenterMax + 1;
	}	
}

void readEEPROM(void) {
	adcCenterMin = eeprom_read_word(EEPROM_CENTER_MIN_ADC);
	adcCenterMax = eeprom_read_word(EEPROM_CENTER_MAX_ADC);
	reCheckCenterAdcValues();
	adcMin = eeprom_read_word(EEPROM_MIN_ADC);
	adcMax = eeprom_read_word(EEPROM_MAX_ADC);
	reCheckAdcValues();
}

void writeEEPROM(void) {
	eeprom_write_word(EEPROM_CENTER_MIN_ADC, adcCenterMin);
	eeprom_write_word(EEPROM_CENTER_MAX_ADC, adcCenterMax);
	eeprom_write_word(EEPROM_MIN_ADC, adcMin);
	eeprom_write_word(EEPROM_MAX_ADC, adcMax);
}

void initADC(void) {
	adcIndex = 0;
	adcCur = ((adcCenterMax + adcCenterMin) >> 1); // center
	adcPrev = 0;
	adcArr = 0;
	adcArrMin = 0x01FF;
	adcArrMax = 0x0000;

	ADMUX = (0 << REFS1) | (0 << REFS0) | // AREF, internal VREF turned off
	        // (1 << ADLAR) | // ADC Left Adjust Result ADCH(8) ADCL(2...)
	        (0 << ADLAR) | // ADC RIGHT Adjust Result ADCH(...2) ADCL(8)
	        (0 << MUX3) | (0 << MUX2) | (0 << MUX1) |
	        (0 << MUX0) | // ADC0 select
	        0x00;

	ADCSRA = (1 << ADEN) |  // ADC Enable
	         (0 << ADSC) |  // NO ADC Start Conversion
	         (1 << ADIF) |  // Reset ADC Interrupt Flag
	         (1 << ADIE) |  // Enable ADC interrupt
#if (F_CPU == 8000000)
			(0 << ADPS2) | (0 << ADPS1) | (1 << ADPS0) | //  ADC Prescale 2 // 8MHz ~ 307692HZ ~ 307KHz
#elif (F_CPU == 16000000)
			 (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0) | //  ADC Prescale 4 // 16MHz ~ 307692HZ ~ 307KHz
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
#elif (F_CPU == 16000000)
	ICR1 = 0x01FF; // 511
#endif

	pwmVal = PWM_CENTER;
	OCR1A = pwmVal;

	TIMSK1 |= (1 << TOIE1); // enable overflow interrupt
}

void initLED(void) {
	LED_DDR |= (1<<LED_BLUE_PIN_NUM)|(1<<LED_GREEN_PIN_NUM)|(1<<LED_RED_PIN_NUM);

	LED_BLUE_ON();
	LED_GREEN_ON();
	LED_RED_ON();
}

ISR(ADC_vect) {
	// NEED ONLY 9 BITS FROM ADC VALUE! Don't use last bit
	// ADC Data register not update until ADCH is read
	// uint16_t adcVal = ((ADCH & 0x01) << 8);
	// adcVal = ADCL;
	uint16_t adcVal = ADCW;
	adcVal >>= 1;

	adcArr += adcVal;
	if(adcArrMin > adcVal) {
		adcArrMin = adcVal;
	}
	if(adcVal > adcArrMax) {
		adcArrMax = adcVal;
	}
	adcIndex++;
	if(adcIndex > 9) {
		adcArr -= (adcArrMin + adcArrMax);
		// adcCur = adcArr / 8;
		adcCur = (adcArr >> 3); // division by 8
		adcArrMin = 0x01FF;
		adcArrMax = 0x0000;
		adcArr = 0;
		adcIndex = 0;
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

uint16_t calMinAdc;
uint16_t calMaxAdc;

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

void calcPwmPerSteps(void) {
	pwmCenterToMaxPerStep = (float)(PWM_MAX - PWM_CENTER) / (float)(adcMax - adcCenterMax);
	pwmCenterToMinPerStep = (float)(PWM_CENTER - PWM_MIN) / (float)(adcCenterMin - adcMin);
}

#if USE_SERIAL
const char str_TriumphShifter[] PROGMEM = "Triumph Shifter\r\n\0";
const char str_MIN[] PROGMEM = "MIN:\0";
const char str_CENTER[] PROGMEM = "CENTER:\0";
const char str_MAX[] PROGMEM = "MAX:\0";
const char str_NL[] PROGMEM = "\r\n\0";
const char str_ADC[] PROGMEM = "ADC:\0";
const char str_PWM[] PROGMEM = "PWM:\0";
const char str_StartCalibrating[] PROGMEM = "Calibrating center...\r\n\0";
const char str_CalibratingMinMax[] PROGMEM = "Calibrating MIN & MAX...\r\n\0";
const char str_SaveToEEPROM[] PROGMEM = "Saving to EEPROM!\r\n\0";
#endif

void setup(void) {
	cli();

	calStep = 0;

	DDRB = DDRC = DDRD = 0x00;
	PORTB = PORTC = PORTD = 0x00;

	initLED();

	initCal();

	_delay_ms(100);

	readEEPROM();

	initADC();

	initPWM();

	if(checkCalIsDown()) {
		calStep = 1; // enter to calibrating
	}

	calcPwmPerSteps();

	LED_BLUE_OFF();
	LED_RED_OFF();

	sei();

	ADCSRA |= (1<<ADIF); // Reset interrupt flag
	ADCSRA |= (1<<ADSC); // Start ADC conversion

#if USE_SERIAL

	uartInit();

	uartSend_P(str_TriumphShifter);

	uartSend_P(str_MIN);
	uartSendUint16_t(adcMin);
	uartSend_P(str_NL);

	uartSend_P(str_CENTER);
	uartSendUint16_t(adcCenterMin);
	uartSendChar('-');
	uartSendUint16_t(adcCenterMax);
	uartSend_P(str_NL);

	uartSend_P(str_MAX);
	uartSendUint16_t(adcMax);
	uartSend_P(str_NL);

	// need time to send 59 bytes
	_delay_ms(200);

	if(calStep) {
		uartSend_P(str_StartCalibrating);
	}

#endif
}

uint16_t calAdcMin = 0x01FF;
uint16_t calAdcMax = 0x0000;

void doCalibrate(void) {

	if(calAdcMin > adcCur) {
		calAdcMin = adcCur;
	} else if(adcCur > calAdcMax) {
		calAdcMax = adcCur;
	}

	if(checkCalIsDown()) {
		if(1 == calStep) {
			adcCenterMin = calAdcMin;
			adcCenterMax = calAdcMax;
			reCheckCenterAdcValues();
#if USE_SERIAL
			uartSend_P(str_CENTER);
			uartSendUint16_t(adcCenterMin);
			uartSendChar('-');
			uartSendUint16_t(adcCenterMax);
			uartSend_P(str_NL);
			uartSend_P(str_CalibratingMinMax);
#endif
			calAdcMin = adcCenterMin - 1;
			calAdcMax = adcCenterMax + 1;
		} else {
			adcMin = calAdcMin;
			adcMax = calAdcMax;
			reCheckAdcValues();
			writeEEPROM();
			calcPwmPerSteps();
			calStep = 0;

#if USE_SERIAL
	uartSend_P(str_MIN);
	uartSendUint16_t(adcMin);
	uartSend_P(str_NL);

	uartSend_P(str_MAX);
	uartSendUint16_t(adcMax);
	uartSend_P(str_NL);

	uartSend_P(str_SaveToEEPROM);
#endif

			calAdcMin = 0x01FF;
			calAdcMax = 0x0000;
			LED_BLUE_OFF();
			return;
		}
		calStep++;
	}
}

void loop(void) {

	static uint32_t secondsPrev = 0;

	if(0 != calStep) {
		if(secondsPrev != seconds) {
			LED_BLUE_TOGGLE();
#if USE_SERIAL
			uartSend_P(str_MIN);
			uartSendUint16_t(calAdcMin);
			uartSendChar(' ');
			uartSend_P(str_MAX);
			uartSendUint16_t(calAdcMax);
			uartSend_P(str_NL);
#endif
		}
		doCalibrate();
	} else {
		if(adcPrev != adcCur) {
			if(adcCur > adcCenterMax) {
				if(adcCur > adcMax) {
					pwmVal = PWM_MAX;
				} else {
					pwmVal = PWM_CENTER + (uint16_t)(pwmCenterToMaxPerStep * (float)(adcCur - adcCenterMax));
				}
			} else if (adcCenterMin > adcCur) {
				if(adcMin > adcCur) {
					pwmVal = PWM_MIN;
				} else {
					pwmVal = PWM_MIN + (uint16_t)(pwmCenterToMinPerStep * (float)(adcCur - adcMin));
				}
			} else {
				pwmVal = PWM_CENTER;
			}
		}
	}

	if(adcPrev != adcCur) {
		adcPrev = adcCur;
	}

	if(secondsPrev != seconds) {
		LED_GREEN_TOGGLE();
#if USE_SERIAL		
		uartSend_P(str_ADC);
		uartSendUint16_t(adcCur);
		uartSendChar(' ');
		uartSend_P(str_PWM);
		uartSendUint16_t(pwmVal);
		uartSend_P(str_NL);
#endif
		secondsPrev = seconds;
	}

}

int main(void) {
	setup();

	while (1) {
		loop();
	}

	
	return 0;
}
