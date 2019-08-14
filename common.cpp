#include <Arduino.h>
#include <Time.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "common.h"

bool raw_measuring = 0;

char *itoa_10lz(char *buf, uint16_t val, uint8_t digits)
{
	uint8_t pos = digits - 1;
	for (uint8_t i = 0; i < digits; i++) {
		buf[pos] = '0' + val % 10;
		val /= 10;
		pos--;
	}

	return buf;
}

char *sprintf_time(char *buf, time_t time)
{
	char *p = buf;

	itoa_10lz(p, year(time), 4);
	p += 4;
	*p++ = '-';
	itoa_10lz(p, month(time), 2);
	p += 2;
	*p++ = '-';
	itoa_10lz(p, day(time), 2);
	p += 2;
	*p++ = ' ';
	itoa_10lz(p, hour(time), 2);
	p += 2;
	*p++ = ':';
	itoa_10lz(p, minute(time), 2);
	p += 2;
	*p++ = ':';
	itoa_10lz(p, second(time), 2);
	p += 2;
	*p = '\0';

	return buf;
}

static char time_buf[STRBUF_TIME_SIZE];

void printTime(time_t time)
{

	Serial.println(sprintf_time(time_buf, time));
}

unsigned short readBattery_mV()
{
	unsigned short adc = analogRead(BATT_MEASURE_PIN);
//	double v;

	return adc * 44 / 10;
//	v = adc * 1.1 / 1024;
//	v *= (820.0 / 200); 

//	return v;
}

void set_bluetooth_mode(enum bt_mode mode)
{
	bluetooth_mode = mode;

	if (mode == BT_PERMANENT) {
		digitalWrite(BT_EN_PIN, HIGH);
		bt_enabled = true;
	}
}


