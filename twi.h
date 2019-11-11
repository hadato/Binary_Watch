#ifndef TWI_H
#define TWI_H

#include <avr/io.h>
#include <stdlib.h>

#define START         0x08
#define MT_SLA_ACK    0x18
#define MT_DATA_ACK   0x28
#define MR_SLA_ACK    0x40
#define MR_DATA_ACK   0x50

#define PULL_UP1 PB1
#define PULL_UP2 PB2

#define TWI_READ  0b00000001
#define TWI_WRITE 0b00000000


typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t days;
	uint8_t weekdays;
	uint8_t months;
	uint8_t years;	
} Time;

class twi
{
	public:
		twi();
		
		/*Hardware access*/		
		void start();
		uint8_t read(uint8_t);		
		void write(uint8_t);				
		uint8_t status();
		void stop();
		
		/*Communication*/
		void PCF_read(uint8_t *, uint8_t);
		void PCF_write(uint8_t *, uint8_t);
		void PCF_writeRegisters(uint8_t address, uint8_t *data, uint8_t count);
		
		/*User Interface*/
		uint8_t set_TimeDate(Time);
		uint8_t update_TimeDate();		
		Time get_TimeDate_BCD();
		Time get_TimeDate_DEC();
		
		
		
	protected:
	private:
		uint8_t LastTimeDate_BCD[7];
		uint8_t LastTimeDate_DEC[7];
		uint8_t BCD2DEC(uint8_t);
		uint8_t BCD2DEC(uint8_t, uint8_t);
		uint8_t DEC2BCD(uint8_t);		
		uint8_t check_TimeDate_BCD(Time);
};

#endif
