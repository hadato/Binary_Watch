/*A source file for multiplexing
*	Timer0 is used for multiplexing
*	Timer1 is used for the button press and general timing needs
*
*
*/

#include "watch.h"
#include <avr/io.h>
#include <stdlib.h>
#include "twi.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>

namespace watch{
	//Variables to store the current time from the RT clock
	Time time_DEC;			//Current time in DEC format
	Time time_BCD;			//Current time in BCD format
	Time time_DEC_new;		//New time to be set on the watch in DEC format
	Time time_BCD_new;		//New time to be set on the watch in BCD format
	uint8_t col;			//A variable to store current column to be displayed - used for multiplexing
	uint16_t press_how_long = 0;	//An internal variable to determine the length of the press
	uint8_t pressed = NOT;			//A variable for the length of the last press. Set to not-pressed
	float battery_level = 0;		//A variable to store the battery level (initialized to 0)
	State state = GO_SLEEP;			//A variable to indicate the current state of the watch. Set to enter the sleep mode
	float u_supply = 0;				//A variable to store the current voltage of the battery
	float u_supply_show = 0;		//A variable to store the voltage of the battery to be displayed on the LEDs
	uint16_t blink_counter = 0;		//A counter for blinking function
	bool blink = true;				//Should the LEDs blink?
	uint8_t error = 0;				//An internal error variable
	uint8_t light = 255;			//A variable to store how much light it is outside (values from 0 (high light) - 255 (low light))
	adc_stat ADC_stat = MEASURE_LIGHT;		//A variable to store the ADC internal state
	
	//Instance of the Twi class - communication with the RT clock
	twi Twi;	
}



ISR(BADISR_vect)
{
	// skip a random interrupt
}

ISR(TIMER2_COMPA_vect){
	//dim the LEDs according to the light
	watch::clear_LED();
}

ISR(TIMER2_COMPB_vect){
	//set the LEDs according to what should be shown
	watch::ISR_handler();		//Turn on the correct LEDs	
}

/********************Functions to handle the bottom interrupts***************************/
ISR (INT0_vect)
{
	PORTB |= LED_ON;
	EIMSK &= ~(1 << INT0);		//Turns off INT0
	TIMSK1 |= (1 << OCIE1A)|(1 << OCIE1B);	//Enable Timer1 on compare match A and B
	TCNT1  = 0;					//Reset timer1 counter
	//watch::press_how_long = 0;
}



ISR (TIMER1_COMPA_vect){
	EIMSK  |= (1 << INT0);		//Turns on INT0
	TIMSK1 &= ~((1 << OCIE1B)|(1 << OCIE1A));	//Disables Timer1 on compare match A and B
	watch::state = GO_SLEEP;
}

ISR (TIMER1_COMPB_vect){
	TCNT1  = 0;				//set the counter to 0	
	
	//Check for the bounce and how long the press holds
	
	//If pressed, increment the counter
	if ((PIND & (1<<PIND2)) == 0){
		watch::press_how_long++;
	}
	//If not pressed, determine how long the press was
	else{if (watch::press_how_long >= EX_LONG_PRESS){
			PORTB |= LED_ON;
			watch::pressed = EX_LONG;
		}else if (watch::press_how_long >= LONG_PRESS){
			PORTB |= LED_ON;
			watch::pressed = LONG;
		}else if (watch::press_how_long >= SHORT_PRESS){
			PORTB |= LED_ON;
			watch::pressed = SHORT;
		}else if (watch::press_how_long < SHORT_PRESS){
			PORTB &= LED_OFF;
			watch::pressed = NOT;
		}
		watch::press_how_long = 0;
		EIMSK  |= (1 << INT0);		//Turns on INT0	
		TIMSK1 &= ~(1 << OCIE1B);	//Disables Timer1 on compare match B
	}
}

ISR(ADC_vect){
	//Interrupt handler for the ADC
	//If measuring battery, save to the battery voltage value else save to the light value
	sei();
	if (watch::ADC_stat == MEASURE_SUPPLY)	{
		watch::u_supply = (1.1*1023/ADC);		// AVcc = Vbg/ADC*1023 = 1.1V*1023/ADC;
		watch::ADC_get_light();
		
		}else{
		watch::ADC_disable();
		watch::light  = ADC>>2;	// save the ADC value for the read on the opto-resistor
	}
}

ISR (TIMER0_COMPA_vect){
	watch::ADC_timer0_stop();
	sei();		//Enable the interrupt being interrupted by other interrupts
	watch::ADC_start();
}


void watch::ADC_timer0_init(){
	// set up timer with CTC mode and prescaler = 64
	TCCR0A |= (1 << WGM01);		//Set CTC mode
	TCCR0B |= (1 << CS01)|(1 << CS00);		//Set prescaler
	
	// initialize counter
	TCNT0 = 0;
	
	// set the frequency to ca 100 Hz
	OCR0A = 155;
}

void watch::ADC_timer0_start(){
	TCNT0	= 0;				//Zero the timer counter
	TIMSK0 |= (1 << OCIE0A);	//Set interrupt on compare A match
}

void watch::ADC_timer0_stop(){
	TIMSK0 &= ~(1 << OCIE0A);	//Disable interrupt on compare A match
}

void watch::ADC_init(){
	ADCSRA  = (1 << ADPS2) | (1 << ADIE);   //Set prescaler to 16 - 62500 Hz and enable interrupts from ADC
	ADMUX  |= (1 <<REFS0);					//Set reference to AVCC
	ADC_timer0_init();						//Initialize timer0 for the delayed measurements of power supply voltage
	
	//ADC
	DDRC  |= (1 << PC2);	// set PC2 as output
	PORTC |= (1 << PC2);	// set PC2 high - turn the power up for the photo-resistor
}

void watch::ADC_enable(){
	ADCSRA |= (1 << ADEN);	//Enable ADC
}

void watch::ADC_disable(){
	ADCSRA &= ~(1 << ADEN);	//Disable ADC
	DDRC   |= (1 << PC3);	//Set PC3 as output
	PORTC  &= ~(1 << PC3);	//Set PC3 low
	PORTC  &= ~(1 << PC2);	//Set PC2 low
}

void watch::ADC_start(){
	ADCSRA |= (1 << ADSC);
}

int watch::ADC_get_supply_light(){
	ADC_enable();
	ADMUX = ADMUX_ADC_VBG;			// set to measure the AVCC using VBG
	ADC_stat = MEASURE_SUPPLY;		// set the ADC status: measure supply voltage
	ADC_timer0_start();				// start the timer to introduce the measurement delay
	return 0;
}

int watch::ADC_get_light(){
	ADC_enable();
	ADMUX = ADMUX_ADC_LIGHT;		// set to measure the voltage on the opto-resistor
	ADC_stat = MEASURE_LIGHT;		// set the ADC status: measure the opto-resistor
	DDRC  &= ~(1 << PC3);			// set PC3 as input - for the opto-resistor measurement
	PORTC |= (1 << PC2);			// set the supply pin PC2 high - turn the power to the opto-resistor
	//_delay_us(10);
	ADC_start();
	return 0;
}




void watch::ISR_handler(){
	watch::col++;						//Increment column count
	if (watch::col == 2) watch::col++;	//Skip 2 since bit 2 is not assigned to the port
	watch::col = watch::col % 8;			//modulo 8 to ensure that col <= 7
	watch::show(watch::col, watch::time_BCD,state);	//show state
	watch::blink_counter = (watch::blink_counter + 1)%BLINK_CNTR_MAX;	//Increment the blink counter and ensure it is in the limits of BLINK_CNTR_MAX
}

State watch::state_machine(uint8_t state, uint8_t pressed){
	/*This function servers as a state automate which determines the state change after a bottom press*/
	/*This function also updates current battery voltage*/
	u_supply_show = u_supply;
	
	State new_state = GO_SLEEP; //Set the new state temporally to GO_SLEEP
	
	//Check the state and go to an appropriate routine accordingly
	switch (state){
		
		
		//If state == AFTER_SLEEP - default from sleep waking.
		case AFTER_SLEEP:
		switch (pressed){
			case SHORT:
			case LONG:
			case EX_LONG:
			new_state = SHOW_TIME;
			break;
		}
		break;
		//If state == SHOW_TIME
		case SHOW_TIME:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SHOW_DATE;
			break;
			case LONG:
			new_state = GO_SLEEP;
			break;
			case EX_LONG:
			new_state = SET_SECONDS;
			//Save the current time to a new variable
			time_BCD_new = time_BCD;
			break;
		}
		break;
		//If state == SHOW_DATE
		case SHOW_DATE:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SHOW_BAT;
			break;
			case LONG:
			new_state = GO_SLEEP;
			break;
			case EX_LONG:
			new_state = SET_SECONDS;
			//Save the current time to a new variable
			time_BCD_new = time_BCD;
			break;
		}
		break;
		//If state == SHOW_BAT
		case SHOW_BAT:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = GO_SLEEP;
			break;
			case LONG:
			new_state = GO_SLEEP;
			break;
			case EX_LONG:
			new_state = SET_SECONDS;
			//Save the current time to a new variable
			time_BCD_new = time_BCD;
			break;
		}
		break;
		case SET_SECONDS:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_SECONDS;
			increment_seconds();
			break;
			case LONG:
			new_state = SET_MINUTES;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			time_BCD = time_BCD_new;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		case SET_MINUTES:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_MINUTES;
			increment_minutes();
			break;
			case LONG:
			new_state = SET_HOURS;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		case SET_HOURS:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_HOURS;
			increment_hours();
			break;
			case LONG:
			new_state = SET_DAYS;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		case SET_DAYS:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_DAYS;
			increment_day();
			break;
			case LONG:
			new_state = SET_MONTHS;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		case SET_MONTHS:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_MONTHS;
			increment_month();
			break;
			case LONG:
			new_state = SET_YEARS;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		case SET_YEARS:
		//Check for how long the bottom was pressed and act accordingly
		switch (pressed){
			case SHORT:
			new_state = SET_YEARS;
			increment_year();
			break;
			case LONG:
			new_state = SHOW_TIME;
			break;
			case EX_LONG:
			new_state = SHOW_TIME;
			set_TimeDate(time_BCD_new);
			break;
		}
		break;
		
		default:
		break;
	}
	return new_state;
}

void watch::update_TimeDate(){
	Twi.update_TimeDate();
	time_DEC = Twi.get_TimeDate_DEC();
	time_BCD = Twi.get_TimeDate_BCD();
}

uint8_t watch::set_TimeDate(Time time){
	return Twi.set_TimeDate(time);
}

uint8_t watch::init_TimeDate(){
	//Set a required time to the
	Time current_time;
	current_time.seconds  = 0b00000000;
	current_time.minutes  = 0b00011001;
	current_time.hours    = 0b00100001;
	current_time.days     = 0b00000110;
	current_time.weekdays = 0b00000000;
	current_time.months   = 0b00010001;
	current_time.years    = 0b00011001;
	
	error = Twi.set_TimeDate(current_time);
	return error;
}

void watch::sentToArduino(uint8_t data){
	//Send to Arduino to check the functionality
	Twi.start();
	Twi.write(0b00001000);
	Twi.write(data);
	Twi.stop();
}

uint8_t watch::BCD2DEC(uint8_t data, uint8_t cntr){
	/*Function to calculate a decimal representation from a BCD format*/
	
	//if the years have to be calculated
	if (cntr == 6){
		return ((data & 0b11110000) >> 4)*10 + (data & 0b00001111);
	}
	return ((data & 0b01110000) >> 4)*10 + (data & 0b00001111);
}

uint8_t watch::BCD2DEC(uint8_t data){
	/*Function to calculate a decimal representation from a BCD format*/
	return ((data & 0b01110000) >> 4)*10 + (data & 0b00001111);
}

uint8_t watch::DEC2BCD(uint8_t data){
	/*Function to calculate BCD value from a decimal format*/
	return (((data/10) << 4) + (data%10));
}

void watch::increment_seconds(){
	time_BCD_new.seconds = DEC2BCD((BCD2DEC(time_BCD_new.seconds)+1)%60);
}
void watch::increment_minutes(){
	time_BCD_new.minutes = DEC2BCD((BCD2DEC(time_BCD_new.minutes)+1)%60);
}
void watch::increment_hours(){
	time_BCD_new.hours = DEC2BCD((BCD2DEC(time_BCD_new.hours)+1)%24);
}
void watch::increment_day(){
	time_BCD_new.days = DEC2BCD((BCD2DEC(time_BCD_new.days))%31+1);
}
void watch::increment_month(){
	time_BCD_new.months = DEC2BCD((BCD2DEC(time_BCD_new.months))%12+1);
}
void watch::increment_year(){
	time_BCD_new.years = DEC2BCD((BCD2DEC(time_BCD_new.years)+1)%50);
}

void watch::timer1_init(){
	// set the timer with prescaler = 256 and CTC mode
	TCCR1B |= (1 << WGM12)|(1 << CS12);
	
	// initialize counter
	TCNT1 = 0;
	
	// set the frequency to
	// f = F_CPU/(prescaler*(1+OCRA))
	// ~200 Hz	
	OCR1B = 10;
	// ~0.125 Hz - fire once in 8s - makes the uC to enter the sleep mode
	OCR1A = 31249;
	
	TIMSK1 |= (1<<OCIE1A)|(1<<OCIE1B);	//Enable interrupts on compare match
	sei();								//Enable global interrupts
}

void watch::timer2_init(){
	//Initialize timer2 for the multiplexing functionality
	TCCR2A |= (1 << WGM21);		// Set the mode to CTC
	TCCR2B |= (1 << CS21);		// set prescaler to 8 and start the timer
	
	TIMSK2 |= (1 << OCIE2A);	//Set interrupt on compare A match
	TIMSK2 |= (1 << OCIE2B);	//Set interrupt on compare B match
	
	OCR2A = 200;				//set the compare value for the interrupt to 100
	OCR2B =	10;					//set the compare value for the interrupt to 10
}

void watch::sleep_init(){
	//select a Power-down mode
	SMCR |= (1<<SM1);
}

void watch::button_init(){
	//Button press
	DDRD  &= ~(1 << PD2);   // set PD2 (PCINT0) to input
	PORTD |= (1 << PD2);    // turn On the Pull-up
	EIMSK |= (1 << INT0);     // Turns on INT0 in a normal operation mode (active low)
}

void watch::write_port(volatile uint8_t *port, uint8_t val, uint8_t mask){
	uint8_t tmpval = *port;			//Get the actual PORT value
	
	//Mask bits which should not be changed and set the port accordingly
	tmpval &= ~mask;				
	tmpval |= (val & mask);
	*port   = tmpval;
}

void watch::init(){
	//Set the initial value for column - used for multiplexing
	col = 0;

	cli();					//disable all interrupts
	
	DDRB |= 0b00111100;		//set PB2-PB5 as outputs
	DDRD |= 0b11111011;		//set PD0, PD1, PD3-PD7 as outputs
	
	clear_LED();			//set the pins so the LEDs are cleared		
	
	//Button press
	button_init();
	
	//ADC initialization for the battery and light measurement
	ADC_init();
		
	//Initialize timer1 for the bottom press and automatic GO_SLEEP after 8s
	timer1_init();
	
	//Initialize timer2 for the multiplexing - LED visualization
	timer2_init();
	
	//Initialize the sleep mode
	sleep_init();
		
	sei();						// enable global interrupts
	
	//Initialize the watch with time to start the RT clock to operate - Uses TWI - interrupts have to be enabled
	init_TimeDate();
}

void watch::set_LED_intensity(uint8_t intensity){
	//A function to set an intensity of the LEDs
	
	//Get the values to reasonable levels (1-23)
	uint8_t intensity_temp = (intensity/255.0)*200 + 1;	
	if (intensity_temp > 199) intensity_temp = 199;	
	//Write to the compare match B register on timer 2
	OCR2B = intensity_temp;
}

uint8_t watch::multiplex(uint8_t col){
	return (0b00000001 << col) & 0b11111011;
}

void watch::clear_LED(){
	WRITE_PORT(PORTB, 0b00111100, 0b00111100);
	WRITE_PORT(PORTD, 0b00000000, 0b11111011);
}

void watch::show(uint8_t col, Time time, State state){
	uint8_t MLTX = multiplex(col);
	uint8_t row  = 0;
	uint8_t batt = 0;
	//Switch between different states
	switch (state){
		case SHOW_TIME:
			blink = false;
			switch (col){
				case 0:
					row = ~((time_BCD.seconds & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);		//Write the correct values to port B, 0 means LED ON!
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 1:
					row = ~((time_BCD.seconds >> 4 & 0b00000111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);					
					break;
				case 2:
					clear_LED();
					break;
				case 3:					
					row = ~((time_BCD.minutes & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);		//Write the correct values to port B, 0 means LED ON!
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 4:					
					row = ~((time_BCD.minutes >> 4 & 0b00000111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 5:
					row = ~((time_BCD.hours & 0b00001111)<<2);
					clear_LED();					
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 6:					
					row = ~((time_BCD.hours >> 4 & 0b00000011)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row , 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 7:
					clear_LED();
					WRITE_PORT(PORTB, ~0b00000100, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
			}		
		break;
		
		case SHOW_DATE:
			blink = false;
			switch (col){
				case 0:					
					row = ~((time_BCD.years & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 1:
					row = ~((time_BCD.years >> 4 & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 2:
					clear_LED();
					break;
				case 3:
					row = ~((time_BCD.months & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 4:
					row = ~((time_BCD.months >> 4 & 0b00000001)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 5:					
					row = ~((time_BCD.days & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 6:					
					row = ~((time_BCD.days >> 4 & 0b00000011)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 7:
					clear_LED();
					WRITE_PORT(PORTB, ~0b00001000, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
			}		
		break;
		
		case SHOW_BAT:
			blink = false;
			batt = 	((uint8_t(u_supply_show) << 4) + (uint8_t(u_supply_show*10)%10));
			switch (col){
				case 0:
					row = ~((batt & 0b00001111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);		//Write the correct values to port B, 0 means LED ON!
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 1:
					row = ~(((batt >> 4) & 0b00000111)<<2);
					clear_LED();
					WRITE_PORT(PORTB, row, 0b00111100);		//Write the correct values to port B, 0 means LED ON!
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				case 7:
					clear_LED();
					WRITE_PORT(PORTB, ~0b00010000, 0b00111100);
					WRITE_PORT(PORTD, MLTX, 0b11111011);
					break;
				default:
					clear_LED();
					break;				
			}
		break;
		
		case SET_SECONDS:
			switch (col){
				case 0:
				blink = true;
				row = ~((time_BCD_new.seconds & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 1:
				blink = true;
				row = ~((time_BCD_new.seconds >> 4 & 0b00000111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 2:
				blink = false;
				clear_LED();
				break;
				case 3:
				blink = false;
				row = ~((time_BCD_new.minutes & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 4:
				blink = false;
				row = ~((time_BCD_new.minutes >> 4 & 0b00000111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 5:
				blink = false;
				row = ~((time_BCD_new.hours & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 6:
				blink = false;
				row = ~((time_BCD_new.hours >> 4 & 0b00000011)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 7:
				blink = false;
				clear_LED();
				set_LED(~0b00000100, MLTX);
				break;
			}
		break;
			
		case SET_MINUTES:
			switch (col){
				case 0:
				blink = false;
				row = ~((time_BCD_new.seconds & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 1:
				blink = false;
				row = ~((time_BCD_new.seconds >> 4 & 0b00000111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 2:
				blink = false;
				clear_LED();
				break;
				case 3:
				blink = true;
				row = ~((time_BCD_new.minutes & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 4:
				blink = true;
				row = ~((time_BCD_new.minutes >> 4 & 0b00000111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 5:
				blink = false;
				row = ~((time_BCD_new.hours & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 6:
				blink = false;
				row = ~((time_BCD_new.hours >> 4 & 0b00000011)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 7:
				blink = false;
				clear_LED();
				set_LED(~0b00000100, MLTX);
				break;
			}
		break;
			
		case SET_HOURS:
		switch (col){
			case 0:
			blink = false;
			row = ~((time_BCD_new.seconds & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 1:
			blink = false;
			row = ~((time_BCD_new.seconds >> 4 & 0b00000111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 2:
			blink = false;
			clear_LED();
			break;
			case 3:
			blink = false;
			row = ~((time_BCD_new.minutes & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 4:
			blink = false;
			row = ~((time_BCD_new.minutes >> 4 & 0b00000111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 5:
			blink = true;
			row = ~((time_BCD_new.hours & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 6:
			blink = true;
			row = ~((time_BCD_new.hours >> 4 & 0b00000011)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 7:
			blink = false;
			clear_LED();
			set_LED(~0b00000100, MLTX);
			break;
		}
		break;
		
		case SET_DAYS:
			blink = false;
			switch (col){
				case 0:
				blink = false;
				row = ~((time_BCD_new.years & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 1:
				blink = false;
				row = ~((time_BCD_new.years >> 4 & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 2:
				blink = false;
				clear_LED();
				break;
				case 3:
				blink = false;
				row = ~((time_BCD_new.months & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 4:
				blink = false;
				row = ~((time_BCD_new.months >> 4 & 0b00000001)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 5:
				blink = true;
				row = ~((time_BCD_new.days & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 6:
				blink = true;
				row = ~((time_BCD_new.days >> 4 & 0b00000011)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 7:
				blink = false;
				clear_LED();
				set_LED(~0b00001000, MLTX);
				break;
			}
		break;
		
		case SET_MONTHS:
			blink = false;
			switch (col){
				case 0:
				blink = false;
				row = ~((time_BCD_new.years & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 1:
				blink = false;
				row = ~((time_BCD_new.years >> 4 & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 2:
				blink = false;
				clear_LED();
				break;
				case 3:
				blink = true;
				row = ~((time_BCD_new.months & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 4:
				blink = true;
				row = ~((time_BCD_new.months >> 4 & 0b00000001)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 5:
				blink = false;
				row = ~((time_BCD_new.days & 0b00001111)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 6:
				blink = false;
				row = ~((time_BCD_new.days >> 4 & 0b00000011)<<2);
				clear_LED();
				set_LED(row, MLTX);
				break;
				case 7:
				blink = false;
				clear_LED();
				set_LED(~0b00001000, MLTX);
				break;
			}
		break;
		
		case SET_YEARS:
		blink = false;
		switch (col){
			case 0:
			blink = true;
			row = ~((time_BCD_new.years & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 1:
			blink = true;
			row = ~((time_BCD_new.years >> 4 & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 2:
			blink = false;
			clear_LED();
			break;
			case 3:
			blink = false;
			row = ~((time_BCD_new.months & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 4:
			blink = false;
			row = ~((time_BCD_new.months >> 4 & 0b00000001)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 5:
			blink = false;
			row = ~((time_BCD_new.days & 0b00001111)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 6:
			blink = false;
			row = ~((time_BCD_new.days >> 4 & 0b00000011)<<2);
			clear_LED();
			set_LED(row, MLTX);
			break;
			case 7:
			blink = false;
			clear_LED();
			set_LED(~0b00001000, MLTX);
			break;
		}
		break;
		
		
		//For other states
		default:
			clear_LED();
			break;
	}
}

void watch::set_LED(uint8_t row, uint8_t column){
	//A function to write a row and column combination into the LEDs and possibly blink the LEDs if needed
	if(blink){
		if (blink_counter < BLINK_CNTR_MAX/2)		{
			WRITE_PORT(PORTB, row, 0b00111100);		//Write the correct values to port B, 0 means LED ON!
			WRITE_PORT(PORTD, column, 0b11111011);
		}
	}else{
		WRITE_PORT(PORTB, row, 0b00111100);			//Write the correct values to port B, 0 means LED ON!
		WRITE_PORT(PORTD, column, 0b11111011);
	}	
}