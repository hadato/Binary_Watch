/*
This is a source file to twi.h to enable two wire communication (I2C) on an Atmega microcontroller
@ Hadato 2019_10_07 
*/
#include "twi.h"
#include <avr/io.h>
//#include <stdlib.h>

/******************************* HARDWARE ACCESS******************************/
twi::twi(){
	/*
	Initialization function - sets the uC to 5 kHz signal clock frequency
	SLC_freq = CPU_clock/(16 + 2(TWBR)*Precsacler_val)
	SLC_freq = 1MHz/(16 + 2*23*4) = 1MHz/200 = 5 kHz	
	Enable the internal pull-ups
	*/
	TWBR = 48;								        // Set bitrate factor to 42
	TWSR |= (0<<TWPS1) | (1<<TWPS0);		        // Set prescaler to 1
	
	//Enable the internal pull-up resistors
	DDRC &= ~(1 << DDC5);				// SCL, Clear the PC5 pin, PC5 is now an input
	PORTC |= (1 << PC5);				// SCL, enable pull-up
	//PORTC &= ~(1 << PC5);				// SCL, turn Off the Pull-up. PC5 is now an input with pull-up disabled
	
	DDRC &= ~(1 << DDC4);				// SDA, Clear the PC4 pin, PC4 is now an input	
	//PORTC &= ~(1 << PC4);				// SDA, turn Off the Pull-up. PC4 is now an input with pull-up disabled
	PORTC |= (1 << PC4);				// SDA, enable pull-up
	
	// Get the last time from the Real-time clock	
	PCF_read(&LastTimeDate_BCD[0], 7);				
}

void twi::start(){
	/*A function to send and check the start condition*/
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);			// Send a start condition
	while(!(TWCR & (1<<TWINT)));			        // Wait for TWINT flag set.	This indicates the start condition has been transmitted		
}

void twi::write(uint8_t data){
	/*A function to send data*/
	TWDR = data;									// Load data to the TWDR register
	TWCR = (1<<TWINT) |	(1<<TWEN);					// Clear TWINT bit in TWCR to start transmission of	address
	while(!(TWCR & (1<<TWINT)));					// Wait for ACK/NACK
}

uint8_t twi::read(uint8_t ack){
	/*A function to read a byte of data sent by the slave.*/
	if(ack != 0) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);    //send an ACK if required
	else TWCR = (1<<TWINT) | (1<<TWEN);				//send NACK
	while (!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t twi::status(){
	/*A function to get a status out of the status register*/
	return TWSR & 0xF8;
}

void twi::stop(){
	/*A function to stop the communication.*/
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);     // End the communication
	//while ((TWCR & (1<<TWSTO)));
}

void twi::enable_pull_ups(){
	//Enable pull-ups on TWI
	PORTC |= (1 << PC5);						// SCL, enable pull-up
	PORTC |= (1 << PC4);						// SDA, enable pull-up
}

void twi::disable_pull_ups(){
	//Disable pull-ups on TWI
	PORTC &= ~(1 << PC5);						// SCL, disable pull-up
	PORTC &= ~(1 << PC4);						// SDA, disable pull-up
}



/**************************** COMMUNICATION ****************************/
void twi::PCF_read(uint8_t *data, uint8_t count){
	/*A function to read the date and time*/
	start();				// Send a start condition
	write(0xA2);			// Send a device address for write
	write(0x02);			// Send an address of the seconds register
	
	stop();					//Send the reset condition
	start();
	write(0xA3);			// Send the device address for read
	
	// Read all the data required
	for (int i = 0; i < count; i++)	{
		*data = read(count-i - 1);
		data++;
	}
	stop();					// Send the stop condition
}

void twi::PCF_write(uint8_t *data, uint8_t count){
	/*A function to set up the required time and date*/
	
	start();				// Send a start condition
	write(0xA2);			// Send a device address for write
	write(0x02);			// Send an address of the seconds register
	
	// Send all the time information needed
	for (int i = 0; i < count; i++){
		write(*data);
		data++;
	}	
	stop();					// Send a stop condition
}

void twi::PCF_writeRegisters(uint8_t address, uint8_t *data, uint8_t count)
{
	/*A function to set up required register value*/
}


/********************************USER INTERFACE****************************/
uint8_t twi::set_TimeDate(Time time){
	/*A function to set a time and date*/
	//Check for the correctness of data, if not acceptable, report an error
	uint8_t error;			           //create a variable to store the error value
	
	//check if the time is correct, if not, report the error
	error = check_TimeDate_BCD(time);  
	if (error) return error;
	
	uint8_t *ptr, data[7];	//Create an array to store time data, create a pointer
	ptr = &time.seconds;	//set the pointer to the first value at time structure
	
	//Save the values from the Time structure time to data array
	for(int i = 0; i < 7; i++){
		data[i] = *(ptr + i);
	}
		
	//Write data to the PCF chip
	PCF_write(data,7);	
	return 0;
}

uint8_t twi::update_TimeDate(){
	/*Function to get a current time from the Real-time clock*/
	uint8_t data[7];		// A variable to save data of length of 7 bytes
	PCF_read(&data[0], 7);	// Read and save the data from the RTC to the the data structure
	
	if(data[0] & 0x80) return 1;  // Check for integrity. If the integrity does not hold, return error
		
	for(int i=0; i < 7; i++) LastTimeDate_BCD[i] = data[i];     // Save data to LastTImeDate_BCD if the integrity holds and return 0
	for(int i=0; i < 7; i++) LastTimeDate_DEC[i] = BCD2DEC(data[i],i);  // Save the data in the DEC format as well
	return 0;
}

Time twi::get_TimeDate_BCD(){
	/*Function to get a time in BCD format*/
	Time time;
	time.seconds = LastTimeDate_BCD[0];
	time.minutes = LastTimeDate_BCD[1];
	time.hours   = LastTimeDate_BCD[2];
	time.days    = LastTimeDate_BCD[3];
	time.weekdays= LastTimeDate_BCD[4];
	time.months  = LastTimeDate_BCD[5];
	time.years   = LastTimeDate_BCD[6];	
	return time;
}

Time twi::get_TimeDate_DEC(){
	/*Function to get time in DEC format*/
	Time time;
	time.seconds = LastTimeDate_DEC[0];
	time.minutes = LastTimeDate_DEC[1];
	time.hours   = LastTimeDate_DEC[2];
	time.days    = LastTimeDate_DEC[3];
	time.weekdays= LastTimeDate_DEC[4];
	time.months  = LastTimeDate_DEC[5];
	time.years   = LastTimeDate_DEC[6];
	return time;	
}

/****************************************PRIVATE FUNCTIONS***********************************************/
uint8_t twi::BCD2DEC(uint8_t data, uint8_t cntr){
	/*Function to calculate a decimal representation from a BCD format*/
		
	//if the years have to be calculated, use all bit in the second DEC digit (0b11110000)
	if (cntr == 6){
		return ((data & 0b11110000) >> 4)*10 + (data & 0b00001111);
	}		
	return ((data & 0b01110000) >> 4)*10 + (data & 0b00001111);
}

uint8_t twi::BCD2DEC(uint8_t data){
	/*Function to calculate a decimal representation from a BCD format*/	
	return ((data & 0b01110000) >> 4)*10 + (data & 0b00001111);
}

uint8_t twi::DEC2BCD(uint8_t data){
	/*Function to calculate BCD value from a decimal format*/
	return (((data/10) << 4) + (data%10));
}

uint8_t twi::check_TimeDate_BCD(Time time){
	/*Function to check if the requested time-date data are in the bounds*/	
	
	//If seconds are corrupted or > 59, report an error
	if ((time.seconds & 0b00001111) > 9 || ((time.seconds & 0b01110000) >> 4) > 5 || BCD2DEC(time.seconds & 0b01111111) > 59){
		return 1;
	}
	//If minutes are corrupted or > 59, report an error
	if ((time.minutes & 0b00001111) > 9 || ((time.minutes & 0b01110000) >> 4) > 5 || BCD2DEC(time.minutes & 0b01111111) > 59){
		return 2;
	}
	//If hours are corrupted or > 23, report an error
	if ((time.hours & 0b00001111) > 9 || ((time.hours & 0b00110000) >> 4) > 2 || BCD2DEC(time.hours) > 23){
		return 3;
	}
	//If days are corrupted or > 31, report an error
	if ((time.days & 0b00001111) > 9 || ((time.days & 0b00110000) >> 4) > 2 || BCD2DEC(time.days) > 31){
		return 4;
	}
	//If weekdays > 6, report an error
	if ((time.weekdays & 0b00000111) > 6){
		return 5;
	}
	//If months are corrupted or > 12, report an error
	if ((time.months & 0b00001111) > 9 || ((time.months & 0b00010000) >> 4) > 1 || BCD2DEC(time.months & 0b00011111) > 12){
		return 6;
	}
	//If years are corrupted, report an error
	if ((time.months & 0b00001111) > 9 || ((time.months & 0b11110000) >> 4) > 9){
		return 7;
	}

	return 0;
}
