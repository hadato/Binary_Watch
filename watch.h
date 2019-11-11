/*
This is a header file to watch.h for the watch based on ATMega 328p chip communicating with a real time clock PCF8563
@ Hadato 2019_11_07
*/

#ifndef WATCH_H
#define WATCH_H

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "twi.h"
#include <avr/sleep.h>

//Define a macro to write to the port - Prioritized for speed during writing
#define WRITE_PORT(port, val, mask)	(port = (port & ~mask) | (val & mask))

enum State{
	SHOW_TIME,
	SHOW_DATE,
	SHOW_BAT, 
	SET_SECONDS,
	SET_MINUTES,
	SET_HOURS,
	SET_DAYS,
	SET_MONTHS,
	SET_YEARS,
	GO_SLEEP,
	AFTER_SLEEP
};

enum Pressed{
	NOT, 
	SHORT, 
	LONG, 
	EX_LONG
};

enum adc_stat {
	MEASURE_SUPPLY,
	MEASURE_LIGHT
};

#define SHORT_PRESS      1
#define LONG_PRESS       100
#define EX_LONG_PRESS    400

#define LED_MASK 0b00000001
#define LED_ON   0b00000001
#define LED_OFF  0b11111110

#define BLINK_CNTR_MAX 260

#define ADMUX_ADC_VBG    ((0 << REFS1)|(1 << REFS0)|(1 << MUX3)|(1 << MUX2)|(1 << MUX1)|(0 << MUX0))
#define ADMUX_ADC_LIGHT  ((0 << REFS1)|(1 << REFS0)|(0 << MUX3)|(0 << MUX2)|(1 << MUX1)|(1 << MUX0))

namespace watch
{	
	//VARIABLES
	extern Time time_DEC;
	extern Time time_BCD;
	extern Time time_DEC_new;
	extern Time time_BCD_new;
	extern uint8_t col;	
	extern uint16_t press_how_long;
	extern uint8_t pressed;
	extern State state;
	extern float battery_level;
	extern float u_supply;
	extern float u_supply_show;
	extern uint16_t blink_counter;		//A counter for blinking function
	extern bool blink;					//Should the LEDs blink?
	extern uint8_t error;
	extern uint8_t light;
	extern adc_stat ACD_stat;
	
	//TWI communication
	extern twi Twi;
	
	//FUNCTIONS
	void init();
	uint8_t multiplex(uint8_t);
	void clear_LED();
	void set_LED(uint8_t row, uint8_t column);	
	void show(uint8_t, Time, State);
	void ISR_handler();	
	void write_port(volatile uint8_t *, uint8_t, uint8_t);
	
	void sleep_init();
	void timer1_init();
	void timer2_init();
	void button_init();
	State state_machine(uint8_t, uint8_t);
	void set_LED_intensity(uint8_t);
	
	//Time set functions
	void update_TimeDate();
	uint8_t init_TimeDate();
	uint8_t set_TimeDate(Time);
	void increment_seconds();
	void increment_minutes();
	void increment_hours();
	void increment_day();
	void increment_month();
	void increment_year();		
	
	//Time format change functions
	uint8_t BCD2DEC(uint8_t, uint8_t);
	uint8_t BCD2DEC(uint8_t);
	uint8_t DEC2BCD(uint8_t);
	
	//Communication with Arduino
	void sentToArduino(uint8_t);
	
	//ADC
	void ADC_timer0_init();
	void ADC_timer0_start();
	void ADC_timer0_stop();
	void ADC_init();
	void ADC_enable();
	void ADC_disable();
	void ADC_start();
	int  ADC_get_light();
	int  ADC_get_supply_light();	
};
#endif