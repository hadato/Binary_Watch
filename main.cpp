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
	cli();
	_delay_ms(500);
	
	
	//Initialize the watch including all its functions	
	watch::init();		
		
	//Turn on all the LEDs for 1s to show that the chip has started working		
	WRITE_PORT(PORTB,0b00000000,0b00111100);
	WRITE_PORT(PORTD,0b11111111,0b11111011);
	
	_delay_ms(2000);
	sei();	
    
    while (1){
		
		
		_delay_ms(10);		
			
		//Update the time from the RT clock and save it to corresponding internal watch variables
		watch::update_TimeDate();
		
		//Measure the voltage and light intensity and set the LED brightness accordingly
 		watch::set_LED_intensity(watch::light);	//Set the intensity of LEDs accordingly
		watch::ADC_get_supply_light();			//Measure the supply voltage and the light intensity
		
		//Send the info to Arduino
		// watch::sentToArduino(watch::light);
		
		//Has to be called in the loop in order to get the state machine to work
		if(watch::pressed){			
			//Change the internal system state			
			watch::state = watch::state_machine(watch::state,watch::pressed);
			watch::pressed = NOT;			
		}
		
		//If the state is set to GO_SLEEP, turn-off all the LEDs and sleep the uC. Wake up and update after the sleep
		if (watch::state == GO_SLEEP){
			_delay_ms(20);
			//Disable all not needed modules and set all LED outputs low
			watch::set_all_pins_low_LED();				// Set all LED outputs low to save energy
			watch::ADC_disable();						// Disable ADC			
			//watch::PRR_disable_all();					// DIsable all modules in the sleep mode
			twi::disable_pull_ups();					// Disable pull-ups on TWI
			
			
			//Prepare for sleep and sleep
			sleep_enable();						//Enable sleep mode
			EIMSK  |= (1 << INT0);				//Turns on INT0 to enable wake up of the uC
			sleep_cpu();						//Go to sleep		
			
			//After sleep
			sleep_disable();
			watch::PRR_enable_ADC_Timer0();		//Turn on just the ADC and Timer0
				
			uint8_t voltage_low = 1;		
			while (voltage_low){
				watch::PRR_enable_ADC_Timer0();	//Turn on just the ADC and Timer0
				watch::ADC_get_supply_light();
				_delay_ms(15);					//Wait for the voltage measurement to complete
				if (watch::u_supply < 2.9){
					//watch::PRR_disable_all();		//Turn off all peripherals
					watch::set_all_pins_low_LED();	// Set all LED outputs low to save energy
					watch::ADC_disable();			// Disable ADC
					sleep_enable();
					sleep_cpu();
					sleep_disable();
				}else{
					voltage_low = 0;
				}
			}
			
			
			cli();						//Disable interrupts to safely change states and PRR register
			watch::PRR_enable_ADC_Timers_TWI();			//Turn on all peripherals except SPI and USART0
			
			
			// After sleep update of states and time from the watch. Disable the sleep, enable the 
			// pull-ups on Twi		
			twi::enable_pull_ups();		//Enable pull-ups on TWI
			
			//Reset internal states after sleep
			watch::pressed = NOT;
			watch::state   = AFTER_SLEEP;
				
			sei();						//Enable interrupts			
			
			//Update time and date and save it to the variables
			watch::set_LED_intensity(200);
			watch::update_TimeDate();					
		}			
    }
}

