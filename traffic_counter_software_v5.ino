

/***************************************************************
 * 
 * Do-It-Yourself TRAFFIC COUNTER
 * Developed by Tomorrow Lab in NYC 2012
 * Developer: Ted Ullricis_measuringh <ted@tomorrow-lab.com>
 * http://tomorrow-lab.com
 * 
 * Materials List:
 * http://bit.ly/OqVIgj
 * 
 * This work is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.
 * Please include credit to Tomorrow Lab in all future versions.
 * 
 ***************************************************************/

#include <Wire.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <DS1307RTC.h>
#include <Time.h>
#include <SimpleTimer.h>
#include <Filters.h>

#include <stdio.h>

#define NOTE_C6  1047
#define NOTE_E6  1319
#define NOTE_G6  1568
#define MEM_SIZE 512 //EEPROM memory size (remaining 2 bytes reserved for count)

// notes in the melody:
int melody[] = {
	NOTE_C6, NOTE_G6};
int noteDurations[] = {
	8,8};
int trigger_value; // pressure reading threshold for identifying a bike is pressing.
int threshold = 9; //change this amount if necessary. tunes sensitivity.
int the_tally; //total amount of sensings.
int incomingByte = 0;   // for incoming serial data
int the_time_offset; // in case of power out, it starts counting time from when the power went out.
int latest_minute;
int the_wheel_delay = 50; //number of milliseconds to create accurate readings for cars. prevents bounce.
int car_timeout = 3000;
long the_wheel_timer; //for considering that double wheel-base of cars, not to count them twice.
int the_max = 0;
int is_measuring = 0;
int count_this = 0;
int strike_number = 0;
float wheel_spacing = 1.100; //average spacing between wheels of car (METERS)
float first_wheel = 0.0000000;
float second_wheel= 0.0000000;
float wheel_time = 0.0000000;
float the_speed = 0.0000000;
int time_slot;
int speed_slot;
int all_speed;

FILE uartstream;

#define RX_BUFSIZE 80

int uart_putchar(char c, FILE *stream)
{
	if (c == '\n')
		Serial.write('\r');
	Serial.write(c);
	return 0;
}

/*
 * Receive a character from the UART Rx.
 *
 * This features a simple line-editor that allows to delete and
 * re-edit the characters entered, until either CR or NL is entered.
 * Printable characters entered will be echoed using uart_putchar().
 *
 * Editing characters:
 *
 * . \b (BS) or \177 (DEL) delete the previous character
 * . ^u kills the entire input buffer
 * . ^w deletes the previous word
 * . ^r sends a CR, and then reprints the buffer
 * . \t will be replaced by a single space
 *
 * All other control characters will be ignored.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).  If the buffer
 * is full (i. e., at RX_BUFSIZE-1 characters in order to keep space for
 * the trailing \n), any further input attempts will send a \a to
 * uart_putchar() (BEL character), although line editing is still
 * allowed.
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int
uart_getchar(FILE *stream)
{
	uint8_t c;
	char *cp, *cp2;
	static char b[RX_BUFSIZE];
	static char *rxp;

	if (rxp == 0)
		for (cp = b;;)
		{
			while (Serial.available() <= 0) {};

			c = Serial.read();
			/* behaviour similar to Unix stty ICRNL */
			if (c == '\r')
				c = '\n';
			if (c == '\n')
			{
				*cp = c;
				uart_putchar(c, stream);
				rxp = b;
				break;
			}
			else if (c == '\t')
				c = ' ';

			if ((c >= (uint8_t)' ' && c <= (uint8_t)'\x7e') ||
					c >= (uint8_t)'\xa0')
			{
				if (cp == b + RX_BUFSIZE - 1)
					uart_putchar('\a', stream);
				else
				{
					*cp++ = c;
					uart_putchar(c, stream);
				}
				continue;
			}

			switch (c)
			{
				case 'c' & 0x1f:
					return -1;

				case '\b':
				case '\x7f':
					if (cp > b)
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;

				case 'r' & 0x1f:
					uart_putchar('\r', stream);
					for (cp2 = b; cp2 < cp; cp2++)
						uart_putchar(*cp2, stream);
					break;

				case 'u' & 0x1f:
					while (cp > b)
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;

				case 'w' & 0x1f:
					while (cp > b && cp[-1] != ' ')
					{
						uart_putchar('\b', stream);
						uart_putchar(' ', stream);
						uart_putchar('\b', stream);
						cp--;
					}
					break;
			}
		}

	c = *rxp++;
	if (c == '\n')
		rxp = 0;

	return c;
}
void raw_print_memory(){

	printf("EEPROM REPORT: \n");
	printf("[");
	for (int i = 0; i <= MEM_SIZE; i++)
	{
		int h = EEPROM.read(i);
		Serial.print(h);
		if (i < MEM_SIZE)
			Serial.print(",");
	}
	printf("]");

}

void erase_memory() {
	//erase current tally
	printf("\nERASING MEMORY ...\n");
	for (int i = 0; i <= MEM_SIZE; i++){
		EEPROM.write(i, 0);
	}  
	the_tally = 0; 
	the_time_offset = 0;
	latest_minute = 0;
}

void printTime(time_t time)
{
	printf("%04d-%02d-%02d %02d:%02d:%02d\n", year(time), month(time), day(time), hour(time), minute(time), second(time));
}

void print_memory() {
	//raw_print_memory();
	if (the_tally > 0) {
		Serial.println("");
		Serial.println("Count , Time (Minutes) , Speed (km/h)");
		for (int i=1; i<= the_tally; i++){
			Serial.print(i);
			Serial.print(" , ");
			long y = EEPROM.read(2*i);
			Serial.print(y);
			Serial.print(" , ");
			long z = EEPROM.read((2*i)+1);
			Serial.println(z); 
			all_speed = (all_speed+z); //add all the speeds together to find average.
			latest_minute = y;    
		}
	}

	Serial.println(""); 
	Serial.print("Total Cars, ");
	Serial.println(the_tally);//read memory
	Serial.print("Total Minutes Measured, ");
	Serial.println(latest_minute);
	Serial.print("Traffic Rate (cars per min), ");
	if ((the_tally/latest_minute) <= 0) {
		Serial.println("0");
	}
	else {
		Serial.println(the_tally/latest_minute);
	}
	Serial.print("Average Car Speed (km per hour), ");
	if ((all_speed/the_tally) <= 0) {
		Serial.println("0");
	}
	else {
		Serial.println(all_speed/the_tally);
	}
	Serial.println("___________________________________________________");
}


void make_tone() {
	for (int thisNote = 0; thisNote < 2; thisNote++) {

		//to calculate the note duration, take one second 
		//divided by the note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000/noteDurations[thisNote];
		tone(13, melody[thisNote],noteDuration);

		//to distinguish the notes, set a minimum time between them.
		//the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;
		delay(pauseBetweenNotes);
		//stop the tone playing:
		noTone(13);
	}
}

int setupRTC() {
	int ret = 0;

	if (!RTC.get()) {
		TimeElements tm;
		// clock probably is stopped, set time to 2018-01-01 00:00:00
		tm.Month = 1;
		tm.Day = 1;
		tm.Year = 2018 - 1970;
		tm.Hour = 0;
		tm.Minute = 0;
		tm.Second = 0;
		if (!RTC.write(tm)) {
			printf("RTC write error\n");
			ret = -1;
		}
	}
	setSyncProvider(RTC.get);
	return ret;
}

SimpleTimer acquireTimer;

uint16_t volatile pressure_avg64_sum = 0;
uint16_t volatile pressure_avg10_sum = 0;
uint16_t volatile pressure_current;
int volatile buf_samples_count = 0;
FilterOnePole biasFilter(LOWPASS, 0.01);


void acquirePressure() {
	static short int last_samples[65];
	static int8_t tail = 0, head = 0;

	int val = analogRead(A0);
	int old64_val = last_samples[tail];
	int old10_val;
	int count;

	pressure_current = val;

	if (head < tail)
		count = (head + 65 - tail) % 65;
	else
		count = (head - tail) % 65;

	biasFilter.input(val);

//	Serial.println(val);
//	Serial.println(biasFilter.output());
//	printf("val: %u\thead: %u\ttail: %u\tcount: %d\tavg64: %u\tavg10: %u\n", val, head, tail, count, pressure_avg64_sum, pressure_avg10_sum);

	last_samples[head] = val;
	head++;
	head = head % 65;

	pressure_avg64_sum += val;
	pressure_avg10_sum += val;

	if (count >= 10) {
		old10_val = head >= 10 ? last_samples[(head - 10) % 65] : last_samples[(head + 65 - 10) % 65];
		pressure_avg10_sum -= old10_val;
	}

	if (count == 64) {
		tail++;
		tail = tail % 65;

		pressure_avg64_sum -= old64_val;
	}

	buf_samples_count = count;
}


void setup() {
	pinMode(A0, INPUT);
	pinMode(2, OUTPUT);
	pinMode(13, OUTPUT);
	analogReference(DEFAULT);
	Serial.begin(115200);

	acquireTimer.setInterval(1, acquirePressure);

	fdev_setup_stream(&uartstream, uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	stdout = &uartstream;
	stdin = &uartstream;

	// make_tone();
	//update the tally variable from memory:
	EEPROM_readAnything(0,  the_tally); //the tally is stored in position 0. assigns the read value to 'the_tally'.
	EEPROM_readAnything((the_tally*2)+1, the_time_offset); //read the last time entry

	if (the_tally < 0) { //for formatting the EEPROM for a new device.
		erase_memory(); 
	}

	//  delay(5000);
	//  analogRead(A0);

	setupRTC();




	Serial.println("Hello, Welcome to the DIY Traffic Counter");
	Serial.println("___________________________________________________");
	if (timeStatus() != timeSet)
		Serial.println("Unable to sync with the RTC"); //синхронизация не удаласть
	else
		Serial.println("RTC has set the system time");

	printf("Current time: ");
	printTime(now());

	Serial.println("");
	Serial.print("Threshold: ");
	Serial.println(threshold);
	Serial.println("___________________________________________________");
	Serial.println("");
	Serial.println("1. PRINT MEMORY");
	Serial.println("2. ERASE MEMORY");
	Serial.println("3. Set Time");
	Serial.println("4. Run raw pressure measurement");
	Serial.println("___________________________________________________");

	biasFilter.setToNewValue(analogRead(A0));

}

void handleMenu() {
	if (Serial.available() > 0) {
		// read the incoming byte:
		incomingByte = Serial.read();
		if (incomingByte == '1') {
			print_memory();
		}
		if (incomingByte == '2') {
			Serial.println("");
			Serial.println("ARE YOU SURE YOU WANT TO ERASE THE MEMORY? Enter Y/N");
		}
		if (incomingByte == 'N' || incomingByte == 'n') {
			Serial.println("MEMORY ERASE CANCELLED");
			Serial.println("___________________________________________________");
		}
		if (incomingByte == 'Y' || incomingByte == 'y') {
			erase_memory();  
			print_memory();
		}
		if (incomingByte == '3') {
			String s;
			unsigned short int y;
			tmElements_t tm;
			while (Serial.available() > 0)
				Serial.read();

			printf("Enter date and time as YYYY-MM-DD HH:MM:SS\n");
			scanf("%04hu-%02hhu-%02hhu %02hhu:%02hhu:%02hhu", &y, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second);

			tm.Year = y - 1970;
			if (!RTC.write(tm))
				printf("RTC write error\n");

			setTime(makeTime(tm));
			printTime(now());
			if (!RTC.read(tm))
				printf("RTC read error\n");

			printTime(makeTime(tm));
		}
		if (incomingByte == '4') {
			while (1) {
				acquireTimer.run();
//				printf("%u %u %u\n", pressure_current, pressure_avg10_sum / 10, pressure_avg64_sum / 64);
			}
		}
	}
}

void loop() {
	acquireTimer.run();

	int val = pressure_current;

	// read local air pressure and create offset.
	trigger_value = pressure_avg64_sum / 64 + threshold;

	//1 - TUBE IS PRESSURIZED INITIALLY
	if ((val > trigger_value) && (buf_samples_count == 64)) {
		Serial.print(pressure_avg10_sum / 10);
		Serial.print(" ");
		Serial.print(pressure_avg64_sum / 64);
		Serial.print(" ");
		Serial.println(val);

		if (strike_number == 0 && is_measuring == 0) { // FIRST HIT
			Serial.println("");
			Serial.println("Car HERE. ");
			printTime(now());
			first_wheel = millis(); 
			is_measuring = 1;
		}
		if (strike_number == 1 && is_measuring == 1) { // SECOND HIT
			Serial.println("Car GONE.");
			printTime(now());
			second_wheel = millis();
			is_measuring = 0;
		}

	}


	//2 - TUBE IS STILL PRESSURIZED
	while( pressure_current > the_max && is_measuring == 1) { //is being pressed, in all cases. to measure the max pressure.
		the_max = pressure_current; 
		acquireTimer.run();
	}


	//3 - TUBE IS RELEASED
	if ( pressure_current < trigger_value - 1 && count_this == 0) { //released by either wheel
		if (strike_number == 0 && is_measuring == 1 && (millis() - first_wheel > the_wheel_delay)) {
			strike_number = 1;
		}
		if (strike_number == 1 && is_measuring == 0 && (millis() - second_wheel > the_wheel_delay) ) {
			count_this = 1;
		}
	}


	//4 - PRESSURE READING IS ACCEPTED AND RECORDED
	if (((pressure_current < trigger_value - 1))
	&& ((count_this == 1 && is_measuring == 0)
	|| ((millis() - first_wheel > car_timeout) && (is_measuring == 1)))) { //has been released for enough time.
		make_tone(); //will buzz if buzzer attached, also LED on pin 13 will flash.
		the_tally++; 
		time_slot = the_tally*2;
		speed_slot = (the_tally*2)+1;
		Serial.print("Pressure Reached = ");
		Serial.println(the_max);
		Serial.print("Current Count = ");
		Serial.println(the_tally);
		// Write the configuration struct to EEPROM
		EEPROM_writeAnything(0, the_tally); //puts the value of x at the 0 address.
		//Serial.print("time between wheels = ");
		wheel_time = ((second_wheel - first_wheel)/3600000);
		//Serial.println(wheel_time);
		int time = ((millis()/1000)/60) + the_time_offset + 1; // the number of seconds since first record.
		EEPROM_writeAnything(time_slot, time); //puts the value of y at address 'the_tally'.
		the_speed = (wheel_spacing/1000)/wheel_time;
		if (the_speed > 0 ) {
			Serial.print("Estimated Speed (km/h) = ");
			Serial.println(the_speed);
			EEPROM_writeAnything(speed_slot, int(the_speed)); //puts the value of y at address 'the_tally'.
		}
		else {
			Serial.println("Speed not measureable");
			EEPROM_writeAnything(speed_slot, 0); //puts the value of y at address 'the_tally'.
		}

		//RESET ALL VALUES
		the_max = 0; 
		strike_number = 0;
		count_this = 0;
		is_measuring = 0;

	}

	handleMenu();

}




