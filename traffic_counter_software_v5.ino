

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

//#define DEBUG_MEMORY

#include <Wire.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <DS1307RTC.h>
#include <Time.h>
#include <TimerOne.h>
#include <Filters.h>
#include <SPI.h>
#include <SdFat.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "uart.h"
#include <stdio.h>

#ifdef DEBUG_MEMORY
#include <MemoryFree.h>
#endif

#define LOCAL_TIME_OFFSET (3 * SECS_PER_HOUR)

#define NOTE_C6  1047
#define NOTE_E6  1319
#define NOTE_G6  1568

#define THRESHOLD 9
#define HYSTERESIS 4

#define LED_PIN 4
#define SPI_SPEED SD_SCK_MHZ(8)


// notes in the melody:
int16_t the_tally; //total amount of sensings.
char incomingByte = 0;   // for incoming serial data
uint16_t the_time_offset; // in case of power out, it starts counting time from when the power went out.
int latest_minute;
const int the_wheel_delay = 60; //number of milliseconds to create accurate readings for cars. prevents bounce.
const int car_timeout = 1000; // 3,8 km/h minimal speed
long the_wheel_timer; //for considering that double wheel-base of cars, not to count them twice.
short unsigned int the_max = 0;
bool is_measuring = 0;
bool count_this = 0;
uint8_t strike_number = 0;
const float wheel_spacing = 1.06; //average spacing between wheels of car (METERS)
float first_wheel = 0.0000000;
float second_wheel= 0.0000000;
short int time_slot;
short int speed_slot;
int all_speed;
const uint8_t sd_CS = 10;
bool raw_measuring = 0;

bool sdReady = 0;
//Sd2Card card;
//SdVolume volume;
//SdFile root;

SdFat sd;
SdFile dataFile;

bool dataFileOpened = 0;
uint8_t dataFileHour;
char logFilename[13]; /* 8.3 filename + \0 */

void setup_sd() {
	Serial.print(F("\nInit SD..."));

	if (sd.begin(sd_CS, SPI_SPEED)) {
		Serial.println(F("SD OK"));
		sdReady = 1;
	} else {
		Serial.println(F("SD init failed"));
		return;
	}

#ifdef DEBUG_MEMORY
	Serial.print(F("freeMemory()="));
	Serial.println(freeMemory());
#endif
}

void raw_print_memory(){

	Serial.println(F("EEPROM REPORT:"));
	Serial.print("[");
	for (int i = 0; i <= E2END; i++)
	{
		int h = EEPROM.read(i);
		Serial.print(h);
		if (i < E2END)
			Serial.print(",");
	}
	Serial.print("]");

}

void erase_memory() {
	//erase current tally
	Serial.println("");
	Serial.println(F("ERASING..."));
	for (int i = 0; i <= E2END; i++){
		EEPROM.write(i, 0);
	}
	the_tally = 0;
	the_time_offset = 0;
	latest_minute = 0;
}

/* wheel_time in ms */
int logToEEPROM(float wheel_time) {
	float the_speed;

	the_tally++;
	time_slot = the_tally * sizeof(uint8_t) * 2;
	speed_slot = (the_tally*sizeof(uint8_t) * 2) + sizeof(uint8_t);
	Serial.print(F("Count = "));
	Serial.println(the_tally);
	Serial.print(F("eeprom addr: "));
	Serial.println(time_slot);

	if (time_slot >= E2END) {
		Serial.println(F("EEPROM FULL"));
		return 1;
	}

	// Write the configuration struct to EEPROM
	EEPROM.put(0, the_tally); //puts the value of x at the 0 address.
	uint8_t time = ((millis()/1000)/60) + the_time_offset + 1; // the number of minutes since first record.
	EEPROM.put(time_slot, time); //puts the value of y at address 'the_tally'.
	the_speed = wheel_spacing/(wheel_time/1000) * 3.6;
	if (the_speed > 0 ) {
		Serial.print("Speed km/h ");
		Serial.println(the_speed);
		EEPROM.put(speed_slot, uint8_t(the_speed)); //puts the value of y at address 'the_tally'.
	}
	else {
		Serial.println("no speed");
		EEPROM.put(speed_slot, uint8_t(0)); //puts the value of y at address 'the_tally' + 1.
	}

	return  0;
}

int sdFileOpen()
{
	if (dataFileOpened && (hour(now()) != dataFileHour)) {
		dataFile.close();
		dataFileOpened = 0;
	}

	if (!dataFileOpened) {
		time_t t = now();

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif
		dataFileHour = hour(t);
		snprintf(logFilename, sizeof(logFilename), "%02u%02u%02u%02u.LOG", year(t) % 100, month(t), day(t), hour(t));

		if (dataFile.open(logFilename, O_CREAT | O_WRITE | O_APPEND)) {
			dataFileOpened = 1;
		} else {
			Serial.println(F("Cannot open logfile"));
			return 1;
		}
	}

	return 0;
}

int logToSd(float wheel_time)
{
	time_t time = now();
	float speed = wheel_spacing/(wheel_time / 1000) * 3.6;
	static char buf[21];

	if (speed < 0)
		speed = 0;

	Serial.print("Speed km/h ");
	Serial.println(speed);

	if (!sdReady) {
		return 1;
	}

	if (sdFileOpen())
		return 1;

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif

	snprintf_P(buf, sizeof(buf), PSTR("%04d-%02d-%02d %02d:%02d:%02d,"), year(time), month(time), day(time), hour(time), minute(time), second(time));

	dataFile.print(time - LOCAL_TIME_OFFSET); /* UTC Unix timestamp */
	dataFile.print(",");
	dataFile.print(buf);
	dataFile.println(speed);
	dataFile.sync();

	return 0;
}

void dumpSdLog()
{
	if (!sdReady)
		return;

	sdFileOpen();

	if (dataFileOpened) {
		dataFile.close();
		dataFileOpened = 0;
	}

	if (!dataFile.open(logFilename, O_READ)) {
		Serial.println(F("Failed to open file"));
		return;
	}

	while (dataFile.available())
		Serial.write(dataFile.read());

	Serial.println("");
	dataFile.close();
}

void printTime(time_t time)
{
/*	Serial.print(year(time));
	Serial.print("-");
	Serial.print(month(time));
	Serial.print("-");
	Serial.print(day(time));
	Serial.print(" ");
	Serial.print(hour(time));
	Serial.print(":");
	Serial.print(minute(time));
	Serial.print(":");
	Serial.println(second(time));
	*/
	printf_P(PSTR("%04d-%02d-%02d %02d:%02d:%02d\n"), year(time), month(time), day(time), hour(time), minute(time), second(time));
}

void print_memory() {
	//raw_print_memory();
	if (the_tally > 0) {
		Serial.println("");
		Serial.println(F("Count , Time (Minutes) , Speed (km/h)"));
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
	Serial.print(F("Total Cars, "));
	Serial.println(the_tally);//read memory
	Serial.print(F("Total Minutes, "));
	Serial.println(latest_minute);
	Serial.print(F("Traffic Rate (cars per min), "));
	if ((the_tally/latest_minute) <= 0) {
		Serial.println("0");
	}
	else {
		Serial.println(the_tally/latest_minute);
	}
	Serial.print("Avg Speed km/h, ");
	if ((all_speed/the_tally) <= 0) {
		Serial.println("0");
	}
	else {
		Serial.println(all_speed/the_tally);
	}
}


void make_tone() {
	digitalWrite(LED_PIN, 1);
	delay(20);
	digitalWrite(LED_PIN, 0);
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
			Serial.println("RTC write error");
			ret = -1;
		}
	}
	setSyncProvider(RTC.get);
	return ret;
}

short unsigned int volatile pressure_current;
FilterOnePole biasFilter(LOWPASS, 0.01);

void acquirePressure() {
	short int val = analogRead(A0);

	pressure_current = val;

	biasFilter.input(val);

	if (raw_measuring)
		printf("%u %u\n", val, (short unsigned int)round(biasFilter.output()));
}

void setupEEPROM() {
	//update the tally variable from memory:
	EEPROM.get(0,  the_tally); //the tally is stored in position 0. assigns the read value to 'the_tally'.
	EEPROM.get((the_tally*2)+1, the_time_offset); //read the last time entry

	if (the_tally < 0) { //for formatting the EEPROM for a new device.
		erase_memory(); 
	}

}

void setup() {
	set_sleep_mode(SLEEP_MODE_IDLE);
	pinMode(A0, INPUT);
	pinMode(LED_PIN, OUTPUT);
	analogReference(INTERNAL);
	analogRead(A0); // reset ADC value after change reference

	Serial.begin(115200);

	digitalWrite(LED_PIN, 1);
	delay(20);
	digitalWrite(LED_PIN, 0);
	delay(200);

	setup_uart();
	setupEEPROM();
	setupRTC();

	digitalWrite(LED_PIN, 1);
	delay(20);
	digitalWrite(LED_PIN, 0);
	delay(200);

	setup_sd();

	digitalWrite(LED_PIN, 1);
	delay(20);
	digitalWrite(LED_PIN, 0);
	delay(200);

	if (timeStatus() != timeSet)
		Serial.println("RTC fail"); //синхронизация не удаласть
	else {
		Serial.println("RTC set");
		digitalWrite(LED_PIN, 1);
		delay(20);
		digitalWrite(LED_PIN, 0);
	}

	printTime(now());

	Serial.println("");
	Serial.print(F("Threshold: "));
	Serial.println(THRESHOLD);
	Serial.println("");
//	Serial.println(F("1. PRINT"));
//	Serial.println(F("2. ERASE"));
	Serial.println(F("3. Set Time"));
	Serial.println(F("4. Run/stop measurement"));
	Serial.println(F("5. Dump current file"));
	Serial.println(F("6. Close all files"));

	biasFilter.setToNewValue(analogRead(A0));
	pressure_current = analogRead(A0);

	Timer1.initialize(1000);
	Timer1.attachInterrupt(acquirePressure);
	sleep_enable();
}

void handleMenu() {
	if (Serial.available() > 0) {
		// read the incoming byte:
		incomingByte = Serial.read();
		if (incomingByte == '1') {
//			print_memory();
		}
		if (incomingByte == '2') {
//			Serial.println("");
//			Serial.println(F("ERASE? Y/N"));
		}
//		if (incomingByte == 'N' || incomingByte == 'n') {
//			Serial.println(F("CANCELLED"));
//		}
//		if (incomingByte == 'Y' || incomingByte == 'y') {
//			erase_memory();  
//			print_memory();
//		}
		if (incomingByte == '3') {
			String s;
			unsigned short int y;
			tmElements_t tm;
			while (Serial.available() > 0)
				Serial.read();

			Serial.println(F("Enter datetime as YYYY-MM-DD HH:MM:SS"));
			scanf_P(PSTR("%04hu-%02hhu-%02hhu %02hhu:%02hhu:%02hhu"), &y, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second);

			tm.Year = y - 1970;
			if (!RTC.write(tm))
				Serial.println(F("RTC error"));

			setTime(makeTime(tm));
			printTime(now());
			if (!RTC.read(tm))
				Serial.println(F("RTC error"));

			printTime(makeTime(tm));
		}
		if (incomingByte == '4')
			raw_measuring = !raw_measuring;

		if (incomingByte == '5')
			dumpSdLog();
		if (incomingByte == '6') {
			if (dataFileOpened) {
				dataFile.close();
				dataFileOpened = 0;
			}
			sdReady = 0;
			Serial.println(F("Card closed"));
		}
	}
}

void loop() {
	short unsigned int val;
	static short unsigned int release_trigger_value = 0;
	short unsigned int trigger_value; // pressure reading threshold for identifying a bike is pressing.

	noInterrupts();
	val = pressure_current;
	// read local air pressure and create offset.
	trigger_value = round(biasFilter.output()) + THRESHOLD;
	interrupts();

	//1 - TUBE IS PRESSURIZED INITIALLY
	if (val >= trigger_value) {
		release_trigger_value = trigger_value - HYSTERESIS;

		if (strike_number == 0 && is_measuring == 0) { // FIRST HIT
			Serial.println("");
			Serial.println(F("Wheel 1"));
			first_wheel = millis(); 
//			printTime(now());
			is_measuring = 1;
		}
		if (strike_number == 1 && is_measuring == 1) { // SECOND HIT
			Serial.println(F("Wheel 2"));
			second_wheel = millis();
//			printTime(now());
			is_measuring = 0;
		}

	}


	//2 - TUBE IS STILL PRESSURIZED
	while( pressure_current > the_max && is_measuring == 1) { //is being pressed, in all cases. to measure the max pressure.
		the_max = pressure_current; 
		sleep_mode();
	}


	//3 - TUBE IS RELEASED
	if ( pressure_current < release_trigger_value && count_this == 0) { //released by either wheel
		if (strike_number == 0 && is_measuring == 1 && (millis() - first_wheel > the_wheel_delay)) {
			strike_number = 1;
		}
		if (strike_number == 1 && is_measuring == 0 && (millis() - second_wheel > the_wheel_delay) ) {
			count_this = 1;
		}
	}


	//4 - PRESSURE READING IS ACCEPTED AND RECORDED
	if (((pressure_current < release_trigger_value))
	&& ((count_this == 1 && is_measuring == 0)
	|| ((millis() - first_wheel > car_timeout) && (is_measuring == 1)))) {
		float wheel_time;
		make_tone();

		wheel_time = second_wheel - first_wheel;

		printTime(now());
		Serial.print(F("max press = "));
		Serial.println(the_max);
//		logToEEPROM(wheel_time);

		logToSd(wheel_time);


		//RESET ALL VALUES
		the_max = 0;
		strike_number = 0;
		count_this = 0;
		is_measuring = 0;

	}

	handleMenu();

	sleep_mode();
}




