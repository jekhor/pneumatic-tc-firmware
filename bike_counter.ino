/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Pneumatic bicycles counter
 * Copyrignt (c) 2019 Yauhen Kharuzhy <jekhor@gmail.com>
 *
 * Inspired by Tomorrow Lab work, http://tomorrow-lab.com, http://bit.ly/OqVIgj
 * (Ted Ullricis_measuringh <ted@tomorrow-lab.com>)
 */

//#define DEBUG_MEMORY

#include <Wire.h>
#include <EEPROM.h>
#include <DS1307RTC.h>
#include <Time.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "uart.h"
#include "LowPassFilter.h"
#include "sd_log.h"
#include "commands.h"
#include "timers.h"

#include "common.h"

#include <stdio.h>

#ifdef DEBUG_MEMORY
#include <MemoryFree.h>
#endif

enum led_state_t {
	LED_OFF,
	LED_ON,
};

static const int debounce_time_ms = 90;
static const int measurement_timeout = 1000; // 3,8 km/h minimal speed

bool is_measuring = 0;
volatile bool bt_enabled = 0;
volatile unsigned long bt_enabled_at = 0;
enum bt_mode bluetooth_mode = BT_AUTO;
uint8_t idle_sleep_mode = SLEEP_MODE_EXT_STANDBY;

static enum led_state_t led_state = LED_OFF;
static uint8_t led_flash_remains = 0;

static void led_timer_cb()
{
	switch (led_state) {
	case LED_OFF:
		digitalWrite(LED_PIN, 1);
		led_state = LED_ON;
		schedule_timer(LED_TIMER, LED_FLASH_ON_MS);
		break;
	case LED_ON:
		digitalWrite(LED_PIN, 0);
		led_state = LED_OFF;
		led_flash_remains--;
		if (led_flash_remains)
			schedule_timer(LED_TIMER, LED_FLASH_OFF_MS);
		break;
	}
}

static void flash_led(uint8_t count)
{
	led_flash_remains += count;

	if (!simple_timers[LED_TIMER].active)
		schedule_timer(LED_TIMER, 0);
}

void setup_led()
{
	simple_timers[LED_TIMER].callback = led_timer_cb;
	simple_timers[LED_TIMER].active = false;
}

int8_t setupRTC() {
	int8_t ret = 0;

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
	/* millis() is broken because of STANDBY sleeping, don't cache time */
	setSyncInterval(0);
	return ret;
}

short unsigned int volatile pressure_current[2] = {0, 0};
LowPassFilter biasFilter0(0);
LowPassFilter biasFilter1(0);

LowPassFilter *biasFilters[NUM_CHANNELS] = {
	&biasFilter0,
	&biasFilter1,
};

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
	}
}

void setupEEPROM() {

}

void btstart_isr() {
	bt_enabled = 1;
	bt_enabled_at = millis();
	digitalWrite(BT_EN_PIN, HIGH);
	digitalWrite(LED_PIN, HIGH);

	idle_sleep_mode = SLEEP_MODE_IDLE;
	schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
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
	if ((bt_enabled
		&& ((digitalRead(BT_STATE_PIN))
			|| (bluetooth_mode == BT_PERMANENT)))
		|| raw_measuring) {
		schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
	} else {
		bt_enabled = 0;
		digitalWrite(BT_EN_PIN, LOW);
		digitalWrite(LED_PIN, LOW);

		idle_sleep_mode = SLEEP_MODE_EXT_STANDBY;
	}
}

void setupPins()
{
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
	pinMode(BT_STATE_PIN, INPUT);
	pinMode(PRESSURE_SENS_EN_PIN, OUTPUT);
	digitalWrite(PRESSURE_SENS_EN_PIN, 1);
}

void reset_channels_state();

void setup() {
	simple_timers[IDLE_TIMEOUT_TIMER].callback = idleTimeout;
	simple_timers[IDLE_TIMEOUT_TIMER].active = false;


	setupPins();

	setup_led();

	delay(10);
	analogReference(INTERNAL);
	analogRead(A0); // reset ADC value after change reference
	analogRead(A1); // reset ADC value after change reference

//	Serial.begin(57600);
	Serial.begin(230400);

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
		Serial.println("RTC fail");
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

	reset_channels_state();

	setupTimer2();
	sleep_enable();
	delay(2);

	PRR |= _BV(PRTIM1);
//	PRR |= _BV(PRTIM2);
//	PRR |= _BV(PRTWI);
//	PRR |= _BV(PRSPI);


//	raw_measuring = true;
//	stop_timer(IDLE_TIMEOUT_TIMER);
}

enum channel_state {
	CH_STATE_IDLE = 0,
	CH_STATE_PRESSED,
	CH_STATE_RELEASED,
};

static uint16_t release_trigger_value[NUM_CHANNELS];
static enum channel_state channels_state[NUM_CHANNELS];

static struct hit hit_series[MAX_HITS];
static uint8_t hit_series_size = 0;

void reset_channels_state()
{
	for (int i = 0; i < NUM_CHANNELS; i++)
		channels_state[i] = CH_STATE_IDLE;
}

void print_hits()
{
	Serial.print("Hit series: ");
	Serial.print(hit_series_size);
	Serial.println(" events:");
	for (int i = 0; i < hit_series_size; i++) {
		Serial.print(hit_series[i].channel);
		Serial.print(":");
		Serial.print(hit_series[i].time);
		Serial.print(" ");
	}
	Serial.println();
}

#define DIV_CEIL(a, b)	((a) / (b) + (((a) % (b)) ? 1 : 0))

void process_hit_series()
{
	int direction;
	float speed_kmh = 0.0;
	float length = 0.0;
	uint8_t bike_count;
	uint8_t channel_count[NUM_CHANNELS];
	uint8_t max_ch_count = 0;

	Serial.println();
	print_hits();

	if (!hit_series_size)
		return;

	for (int i = 0; i < NUM_CHANNELS; i++)
		channel_count[i] = 0;

	/*
	* direction:
	* 0 - unknown
	* 1 - from channel 0 to channel 1
	* 2 - from channel 1 to channel 0
	*/
	if ((hit_series_size == 1)
	    || (hit_series[0].channel == hit_series[1].channel))
		direction = 0;
	else
		direction = hit_series[0].channel + 1;

	if (hit_series_size >= 2) {
		uint16_t wheel_time = 0;

		for (int i = 0; i< hit_series_size; i++)
			if (hit_series[i].channel != hit_series[0].channel) {
				wheel_time = hit_series[1].time - hit_series[0].time;
				break;
			}

		if (wheel_time) {
			float speed_mps;

			speed_mps = sensors_spacing / ((float)wheel_time / 1000);

			/* Calculate vehicle length if it has two wheels and they are
			 * registered both
			 */
			if ((hit_series_size > 2) && (hit_series_size <= 4)) {
				uint16_t time_ms;

				for (int i = 1; i < hit_series_size; i++)
					if (hit_series[i].channel == hit_series[0].channel) {
						time_ms = hit_series[i].time - hit_series[0].time;
						length = speed_mps * time_ms / 1000;
						break;
					}
			}
			speed_kmh = speed_mps * 3.6;
		}
	}

	for (int i = 0; i < hit_series_size; i++)
		channel_count[hit_series[i].channel]++;

	for (int i = 0; i < NUM_CHANNELS; i++)
		if (max_ch_count < channel_count[i])
			max_ch_count = channel_count[i];

	bike_count = DIV_CEIL(max_ch_count, 2);

	/* Don't save known-invalid speed */
	if ((bike_count != 1) || (!isfinite(speed_kmh))) {
		speed_kmh = 0;
		length = 0;
	}

	time_t time = now();
	logToSd(bike_count, direction, speed_kmh, length, time, !raw_measuring);
	logToSdRaw(time, hit_series, hit_series_size);

	flash_led(bike_count);

	hit_series_size = 0;
}

int submit_hit_event(uint8_t ch, unsigned long ts)
{
	static unsigned long first_ts;

	if (!raw_measuring)
		Serial.print(".");

	if (hit_series_size == MAX_HITS)
		return -1;

	if (hit_series_size == 0)
		first_ts = ts;

	hit_series[hit_series_size].time = ts - first_ts;
	hit_series[hit_series_size].channel = ch;

	hit_series_size++;

	return 0;
}

bool check_event(int ch)
{
	static unsigned long hit_time[NUM_CHANNELS];
	static unsigned long release_time[NUM_CHANNELS];
	unsigned long now = millis();
	uint16_t trigger_value;
	uint16_t pressure;
	bool ret = false;

	noInterrupts();
	pressure = pressure_current[ch];
	trigger_value = biasFilters[ch]->output() + THRESHOLD;
	interrupts();

	switch (channels_state[ch]) {
	case CH_STATE_IDLE:
		if (pressure >= trigger_value) {
			/* Hit! */
			release_trigger_value[ch] = trigger_value - HYSTERESIS;
			hit_time[ch] = now;
			is_measuring = 1;
			channels_state[ch] = CH_STATE_PRESSED;
			ret = true;
		}
		break;

	case CH_STATE_PRESSED:
		if (pressure < release_trigger_value[ch]) {
			release_time[ch] = now;
			submit_hit_event(ch, hit_time[ch]);
			channels_state[ch] = CH_STATE_RELEASED;
			ret = true;
		}
		break;

	case CH_STATE_RELEASED:
		/* use substraction to handle millis() overflow correctly */
		if (now - debounce_time_ms > release_time[ch])
			channels_state[ch] = CH_STATE_IDLE;
		break;
	}

	return ret;
}

static void check_for_poweroff(void)
{
	if ((digitalRead(BT_START_PIN) == LOW)
	    && (millis() - bt_enabled_at > BT_EN_POWEROFF_TIMEOUT_MS)) {
		digitalWrite(LED_PIN, 1);
		delay(1000);
		digitalWrite(PWR_ON_PIN, 0);
	}
}

static bool channels_idle()
{
	bool idle = true;

	for (int i = 0; i < NUM_CHANNELS; i++)
		if (channels_state[i] != CH_STATE_IDLE)
			idle = false;

	return idle;
}

void loop() {
	static unsigned long last_hit_time = 0;
	static unsigned char current_sleep_mode = SLEEP_MODE_EXT_STANDBY;
	bool hit;

	hit = check_event(0);
	hit = check_event(1) || hit;

	if (hit) {
		current_sleep_mode = SLEEP_MODE_IDLE;
		last_hit_time = millis();
	} else if (is_measuring
		&& channels_idle()
		&& (millis() - measurement_timeout >= last_hit_time)) {
		process_hit_series();
		is_measuring = 0;
	}

	/* Turn on extended sleep mode if we are in idle state */
	if (!is_measuring)
		current_sleep_mode = idle_sleep_mode;

	processSerial();

	/*
	* Try to execute timers. If any timer remains active, don't enter to
	* extended standby mode, go to idle mode.
	*/
	if (try_timers())
		current_sleep_mode = SLEEP_MODE_IDLE;

	Serial.flush();

	check_for_poweroff();

	set_sleep_mode(current_sleep_mode);
	sleep_mode();
}
