

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
#include <DS1307RTC.h>
#include <Time.h>
#include <SPI.h>
#include <SdFat.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "uart.h"
#include "LowPassFilter.h"
#include <stdio.h>

#ifdef DEBUG_MEMORY
#include <MemoryFree.h>
#endif

#define ARRAY_SIZE(_a)	(sizeof(_a) / sizeof((_a)[0]))

#define LOCAL_TIME_OFFSET (3 * SECS_PER_HOUR)

#define THRESHOLD 25
#define HYSTERESIS 6

#define LED_PIN			4
#define PRESSURE_SENS_EN_PIN	3
#define PWR_ON_PIN		8
#define BT_EN_PIN		17
#define BT_STATE_PIN		9
#define BT_START_PIN		2
#define SPI_SPEED SD_SCK_MHZ(8)
#define IDLE_TIMEOUT_MS		60000
#define BATT_MEASURE_PIN	A6


struct simple_timer {
	bool active;
	unsigned long scheduled_at;
	unsigned long timeout;
	void (*callback)(void);
};

typedef struct simple_timer simple_timer_t;

char incomingByte = 0;   // for incoming serial data
const int the_wheel_delay = 70; //number of milliseconds to create accurate readings for cars. prevents bounce.
const int car_timeout = 1000; // 3,8 km/h minimal speed
short unsigned int the_max[2];
bool is_measuring = 0;
bool count_this = 0;
uint8_t strike_number = 0;
const float wheel_spacing = 1.06; //average spacing between wheels of car (METERS)
float first_wheel = 0.0000000;
float second_wheel= 0.0000000;
const uint8_t sd_CS = 10;
bool raw_measuring = 0;
volatile bool bt_enabled = 0;
uint8_t idle_sleep_mode = SLEEP_MODE_EXT_STANDBY;

#define NUM_TIMERS		1
#define IDLE_TIMEOUT_TIMER	0

simple_timer_t simple_timers[NUM_TIMERS];

bool sdReady = 0;

SdFat sd;
SdFile dataFile;

bool dataFileOpened = 0;
uint8_t dataFileHour;
char logFilename[13]; /* 8.3 filename + \0 */

void schedule_timer(uint8_t timer_id, unsigned long timeout) {
	simple_timers[timer_id].scheduled_at = millis();
	simple_timers[timer_id].timeout = timeout;
	simple_timers[timer_id].active = true;
}

void stop_timer(uint8_t timer_id) {
	simple_timers[timer_id].active = false;
}

void try_timers(void) {
	int i;
	unsigned long t = millis();

	for (i = 0; i < NUM_TIMERS; i++) {
		if (simple_timers[i].active
		    && (t - simple_timers[i].scheduled_at >= simple_timers[i].timeout)) {
			simple_timers[i].active = false;
			simple_timers[i].callback();
		}
	}
}

unsigned short readBattery_mV() {
	unsigned short adc = analogRead(BATT_MEASURE_PIN);
	double v;

	return adc * 44 / 10;
//	v = adc * 1.1 / 1024;
//	v *= (820.0 / 200); 

//	return v;
}


void setup_sd() {
	Serial.print(F("\nInit SD..."));

	if (sd.begin(sd_CS, SPI_SPEED)) {
		Serial.println(F("SD OK"));
		sdReady = 1;
	} else {
		Serial.println(F("SD init failed"));
		sd.initErrorPrint();
		return;
	}

#ifdef DEBUG_MEMORY
	Serial.print(F("freeMemory()="));
	Serial.println(freeMemory());
#endif
}

int sdFileOpen()
{
	time_t t = now();

	if (dataFileOpened && (hour(t) != dataFileHour)) {
		dataFile.close();
		dataFileOpened = 0;
	}

	if (!dataFileOpened) {

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif
		dataFileHour = hour(t);
		snprintf_P(logFilename, sizeof(logFilename), PSTR("%02u%02u%02u%02u.LOG"), year(t) % 100, month(t), day(t), hour(t));

		Serial.println(logFilename);

		if (dataFile.open(logFilename, O_CREAT | O_WRITE | O_APPEND)) {
			dataFileOpened = 1;
		} else {
			Serial.println(F("Cannot open logfile"));
			return 1;
		}
	}

	return 0;
}

int logToSd(float wheel_time, time_t time)
{
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
	dataFile.print(speed);
	dataFile.print(",");
	dataFile.println(readBattery_mV());
	dataFile.sync();

	return 0;
}

char dumpSdLog(char *file)
{
	if (!sdReady)
		return 1;

	sdFileOpen();

	if (dataFileOpened) {
		dataFile.close();
		dataFileOpened = 0;
	}

	if (!dataFile.open(file, O_READ)) {
		Serial.println(F("Failed to open file"));
		return 1;
	}

	while (dataFile.available())
		Serial.write(dataFile.read());

	Serial.println("");
	dataFile.close();

	return 0;
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

void make_tone() {
	digitalWrite(LED_PIN, 1);
	delay(10);
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
	setSyncInterval(0); /* millis() are broken because of STANDBY sleeping, don't cache time */
	return ret;
}

short unsigned int volatile pressure_current[2] = {0, 0};
LowPassFilter biasFilter0(0);
LowPassFilter biasFilter1(0);

void acquirePressure() {
	short int val[2];

	digitalWrite(PRESSURE_SENS_EN_PIN, 0);
	delayMicroseconds(70);

	val[0] = analogRead(A0);
	val[1] = analogRead(A1);
	digitalWrite(PRESSURE_SENS_EN_PIN, 1);

	if (!pressure_current[0])
		biasFilter0.setOutput(val[0]);

	if (!pressure_current[1])
		biasFilter1.setOutput(val[1]);

	pressure_current[0] = val[0];
	pressure_current[1] = val[1];

	if (!is_measuring) {
		biasFilter0.input(val[0]);
		biasFilter1.input(val[1]);
	}

	if (raw_measuring) {
		Serial.print(val[0]);
		Serial.print(' ');
		Serial.print(val[1]);
		Serial.print(' ');
		Serial.print((short unsigned int)biasFilter0.output());
		Serial.print(' ');
		Serial.println((short unsigned int)biasFilter1.output());
//		printf_P(PSTR("%u %u %u %u\n"), val[0], val[1], (short unsigned int)biasFilter0.output(), (short unsigned int)biasFilter1.output());
	}
}

void setupEEPROM() {

}

void btstart_isr() {
	bt_enabled = 1;
	digitalWrite(BT_EN_PIN, HIGH);
	digitalWrite(LED_PIN, 1);

	idle_sleep_mode = SLEEP_MODE_IDLE;
	//		digitalWrite(LED_PIN, 1);
	if (!raw_measuring)
		schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
	else
		stop_timer(IDLE_TIMEOUT_TIMER);
}

ISR(TIMER2_COMPA_vect) {
//ISR(TIMER2_OVF_vect) {
	acquirePressure();
}

void setupTimer2() {
	TCCR2A = _BV(WGM21); // reset counter on compare match
	TCCR2B = _BV(CS22) | _BV(CS20); // clk/128 = 62500 Hz
//	OCR2A = 208;
	OCR2A = 255;
	TIMSK2 |= _BV(OCIE2A); // ~299 Hz
//	TIMSK2 |= _BV(TOIE2); // ~299 Hz
}

void idleTimeout(void) {
	bt_enabled = 0;
	digitalWrite(BT_EN_PIN, LOW);
	digitalWrite(LED_PIN, LOW);

	idle_sleep_mode = SLEEP_MODE_EXT_STANDBY;
//	digitalWrite(LED_PIN, 0);
}

void print_help(void) {
	puts_P(PSTR(
	"Commands:\n"
	" time [yyyy-mm-dd HH:MM:SS] \t - get/set time\n"
	" raw\t\t\\tt - toggle raw dump\n"
	" ls\t\t\t\t - list files on SD\n"
	" dump [file]\t\t\t - dump file content\n"
	" batt\t\t\t\t - show battery voltage\n"
	));
}

void setup() {
	simple_timers[IDLE_TIMEOUT_TIMER].callback = idleTimeout;
	simple_timers[IDLE_TIMEOUT_TIMER].active = false;

	pinMode(PWR_ON_PIN, OUTPUT);
	digitalWrite(PWR_ON_PIN, 1);
	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(BATT_MEASURE_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);
	pinMode(BT_START_PIN, INPUT_PULLUP);
	pinMode(BT_EN_PIN, OUTPUT);
	digitalWrite(BT_EN_PIN, 0);
	attachInterrupt(digitalPinToInterrupt(BT_START_PIN), btstart_isr, FALLING);
	pinMode(PRESSURE_SENS_EN_PIN, OUTPUT);
	digitalWrite(PRESSURE_SENS_EN_PIN, 1);
	delay(10);
	analogReference(INTERNAL);
	analogRead(A0); // reset ADC value after change reference
	analogRead(A1); // reset ADC value after change reference

	Serial.begin(115200);

	digitalWrite(LED_PIN, 1);
	delay(20);
	digitalWrite(LED_PIN, 0);
	delay(200);

	setup_uart();
	Serial.println(F("Build: " __DATE__ " " __TIME__));
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
	print_help();

	setupTimer2();
	sleep_enable();
	delay(2);

	PRR |= _BV(PRTIM1);
//	PRR |= _BV(PRTIM2);
//	PRR |= _BV(PRTWI);
//	PRR |= _BV(PRSPI);
}


char cmd_time(char argc, char *argv[]) {


	String s;
	unsigned short int y;
	tmElements_t tm;

	if ((argc == 1) || (argc > 2)) {
		Serial.println(F("Usage: time YYYY-MM-DD HH:MM:SS"));
		return 1;
	}

	if (argc == 2) {
		sscanf_P(argv[0], PSTR("%04hu-%02hhu-%02hhu"), &y, &tm.Month, &tm.Day);
		sscanf_P(argv[1], PSTR("%02hhu:%02hhu:%02hhu"), &tm.Hour, &tm.Minute, &tm.Second);

		tm.Year = y - 1970;
		if (!RTC.write(tm))
			Serial.println(F("RTC error"));

		setTime(makeTime(tm));
		if (!RTC.read(tm))
			Serial.println(F("RTC error"));

	}
	printTime(now());

	return 0;
}

char cmd_ls(char argc, char *argv[]) {
	if (!sdReady)
		return 1;

	sd.ls(LS_R);

	return 0;
}

char cmd_dump(char argc, char *argv[]) {
	if (argc == 1)
		return dumpSdLog(argv[0]);
	else
		return dumpSdLog(logFilename);

	return 0;
}

char cmd_raw(char argc, char *argv[]) {
	raw_measuring = !raw_measuring;

	if (raw_measuring) {
		if (raw_measuring)
			stop_timer(IDLE_TIMEOUT_TIMER);
		else
			schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
	}

	return 0;
}

char cmd_batt(char argc, char *argv[]) {
	Serial.println(readBattery_mV());

	return 0;
}

char cmd_poff(char argc, char *argv[]) {
	digitalWrite(LED_PIN, 1);
	digitalWrite(PWR_ON_PIN, 0);
	return 0;
}

typedef char (*cmd_handler_t)(char argc, char *argv[]);

struct cmd_table_entry {
	PGM_P cmd;
	cmd_handler_t handler;
};

const char cmd_time_name[] PROGMEM = "time";
const char cmd_ls_name[] PROGMEM = "ls";
const char cmd_dump_name[] PROGMEM = "dump";
const char cmd_raw_name[] PROGMEM = "raw";
const char cmd_poff_name[] PROGMEM = "poff";
const char cmd_batt_name[] PROGMEM = "batt";

const struct cmd_table_entry cmd_table[] = {
	{
		cmd_time_name,
		cmd_time,
	},
	{
		cmd_ls_name,
		cmd_ls,
	},
	{
		cmd_dump_name,
		cmd_dump,
	},
	{
		cmd_raw_name,
		cmd_raw,
	},
	{
		cmd_poff_name,
		cmd_poff,
	},
	{
		cmd_batt_name,
		cmd_batt,
	},
};

#define MAX_CMD_ARGS	3

void parse_cmdline(char *buf, uint8_t len) {
	char *cmd = strtok(buf, " ");
	char *argv[MAX_CMD_ARGS];
	uint8_t argc = 0;
	unsigned char i;
	char ret = 1;

	if (!cmd)
		return;

	while (argc < MAX_CMD_ARGS) {
		argv[argc] = strtok(NULL, " ");
		if (argv[argc])
			argc++;
		else
			break;
	}

	for (i = 0; i < ARRAY_SIZE(cmd_table); i++) {
		if (!strcmp_P(cmd, cmd_table[i].cmd)) {
			ret = cmd_table[i].handler(argc, argv);
			break;
		}
	}

	if (ret)
		Serial.println("ERROR");

}

#define SERIAL_BUF_LEN	80

static char serial_buf[SERIAL_BUF_LEN];
static uint8_t serial_buf_level = 0;

void processSerial() {
	char c;

	while (Serial.available()) {

		idle_sleep_mode = SLEEP_MODE_IDLE;
//		digitalWrite(LED_PIN, 1);
		if (!raw_measuring)
			schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
		else
			stop_timer(IDLE_TIMEOUT_TIMER);

		c = Serial.read();
		Serial.write(c);

		digitalWrite(LED_PIN, 1);
		if ((c == '\r') || (c == '\n')) {
			if (c == '\r')
				Serial.write('\n');

			serial_buf[serial_buf_level] = '\0';
			serial_buf_level++;

			parse_cmdline(serial_buf, serial_buf_level);

			serial_buf_level = 0;
			Serial.write("> ");
		} else {
			if (serial_buf_level < SERIAL_BUF_LEN - 1) {
				serial_buf[serial_buf_level] = c;
				serial_buf_level++;
			}
		}
	}
		digitalWrite(LED_PIN, 0);
}

void loop() {
	short unsigned int val;
	static short unsigned int release_trigger_value = 0;
	short unsigned int trigger_value; // pressure reading threshold for identifying a bike is pressing.
	static unsigned char current_sleep_mode = SLEEP_MODE_EXT_STANDBY;

	noInterrupts();
	val = pressure_current[0];
	// read local air pressure and create offset.
	trigger_value = biasFilter0.output() + THRESHOLD;
	interrupts();

	if (!is_measuring)
		current_sleep_mode = idle_sleep_mode;

	//1 - TUBE IS PRESSURIZED INITIALLY
	if (val >= trigger_value) {
		release_trigger_value = trigger_value - HYSTERESIS;
		current_sleep_mode = SLEEP_MODE_IDLE;
		set_sleep_mode(current_sleep_mode);

		if (strike_number == 0 && is_measuring == 0) { // FIRST HIT
			if (!raw_measuring) {
				Serial.println("");
				Serial.println(F("W1"));
			}
			first_wheel = millis(); 
			is_measuring = 1;
			the_max[0] = val;
		}
		if (strike_number == 1 && is_measuring == 1) { // SECOND HIT
			if (!raw_measuring) {
				Serial.println(F("W2"));
			}
			second_wheel = millis();
			is_measuring = 0;
			the_max[1] = val;
		}

	}


	//2 - TUBE IS STILL PRESSURIZED
	while( val > the_max[strike_number] && is_measuring == 1) { //is being pressed, in all cases. to measure the max pressure.
		the_max[strike_number] = val; 
		sleep_mode();
	}


	//3 - TUBE IS RELEASED
	if ( val < release_trigger_value && count_this == 0) { //released by either wheel
		if (strike_number == 0 && is_measuring == 1 && (millis() - first_wheel > the_wheel_delay)) {
			strike_number = 1;
		}
		if (strike_number == 1 && is_measuring == 0 && (millis() - second_wheel > the_wheel_delay) ) {
			count_this = 1;
		}
	}


	//4 - PRESSURE READING IS ACCEPTED AND RECORDED
	if (((val < release_trigger_value))
	&& ((count_this == 1 && is_measuring == 0)
	|| ((millis() - first_wheel > car_timeout) && (is_measuring == 1)))) {
		float wheel_time;
		time_t time = now();
		make_tone();

		wheel_time = second_wheel - first_wheel;

		if (!raw_measuring) {
			printTime(time);
			Serial.print(F("max press = "));
			Serial.print(the_max[0]);
			Serial.print(" ");
			Serial.println(the_max[1]);
			Serial.print("avg: ");
			Serial.println(biasFilter0.output());
		}

		logToSd(wheel_time, time);

		//RESET ALL VALUES
		strike_number = 0;
		count_this = 0;
		is_measuring = 0;
		current_sleep_mode = idle_sleep_mode;

	}

	processSerial();

	try_timers();

	Serial.flush();

	set_sleep_mode(current_sleep_mode);
	sleep_mode();
}




