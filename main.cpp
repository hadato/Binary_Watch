/*
 * BinaryWatch_v0.1.cpp
 * Blink Test
 * Created: 7.10.2019 10:56:57
 * Author : Hadato
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "twi.h"
#include "watch.h"

int main(void){		
	//Wait for the power to get settled
	_delay_ms(500);
	
	//Initialize status LED for Blink
	DDRB |= 0b00000001;		//set PB0 as output
	PORTB = 0b00000001;		//set PB0 to 1
		
	//Initialize the watch including all its functions
	watch::init();
			
	//Turn on all the LEDs for 1s to show that the chip has started working
	cli();	
	WRITE_PORT(PORTB,0b00000000,0b00111100);
	WRITE_PORT(PORTD,0b11111111,0b11111011);
	_delay_ms(1000);
	sei();	
    
    while (1){
		_delay_ms(10);			
		//Update the time from the RT clock and save it to corresponding internal watch variables
		watch::update_TimeDate();
		
		//Measure the voltage and light intensity and set the LED brightness accordingly
 		watch::set_LED_intensity(watch::light);	//Set the intensity of LEDs accordingly
		watch::ADC_get_supply_light();			//Measure the supply voltage and the light intensity
		
		//Has to be called in the loop in order to get the state machine to work
		if(watch::pressed){			
			//Change the internal system state			
			watch::state = watch::state_machine(watch::state,watch::pressed);
			watch::pressed = NOT;			
		}
		
		//If the state is set to GO_SLEEP, turn-off all the LEDs and sleep the uC. Wake up and update after the sleep
		if (watch::state == GO_SLEEP){
			_delay_ms(20);
			//Turn off all LEDs
			PORTB  = 0b00000000;
			watch::clear_LED();			
			
			//Prepare for sleep and sleep
			sleep_enable();				//Enable sleep mode
			EIMSK  |= (1 << INT0);		//Turns on INT0 to enable wake up of the uC
			sleep_cpu();				//Go to sleep		
			
			//After sleep update of states and time from the watch. Disable the sleep
			cli();						//Disable interrupts to safely update the states
			
			//Reset internal states after sleep
			watch::pressed = NOT;
			watch::state   = AFTER_SLEEP;
			
			//Disable sleep after waking up
			sleep_disable();
			sei();						//Enable interrupts			
			
			//Update time and date and save it to the variables
			watch::update_TimeDate();					
		}				
    }
}

