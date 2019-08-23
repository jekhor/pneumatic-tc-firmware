
#include <SPI.h>
#include <SdFat.h>
#include <errno.h>
#include "sd_log.h"
#include "common.h"

#define SPI_SPEED SD_SCK_MHZ(8)
const uint8_t sd_CS = 10;

static bool sdReady = 0;

SdFat sd;
static SdFile dataFile;
static SdFile rawLogFile;

static bool dataFileOpened = 0;
static bool rawFileOpened = 0;
static uint8_t dataFileHour;
static char logFilename[13]; /* 8.3 filename + \0 */

int isSdReady(void)
{
	return sdReady;
}

char *currentLogFilename(void)
{
	return logFilename;
}

void setup_sd()
{
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

static char *make_filename(char *buf, time_t time)
{
	char *p = buf;

	itoa_10lz(p, year(time) % 100, 2);
	p += 2;
	itoa_10lz(p, month(time), 2);
	p += 2;
	itoa_10lz(p, day(time), 2);
	p += 2;
	itoa_10lz(p, hour(time), 2);
	p += 2;
	p[0] = '.';
	p[1] = 'C';
	p[2] = 'S';
	p[3] = 'V';
	p[4] = '\0';

	return buf;
}

int sdFileOpen()
{
	time_t t = now();

	if (dataFileOpened && (hour(t) != dataFileHour)) {
		dataFile.close();
		dataFileOpened = 0;
	}

	if (!dataFileOpened) {
		bool exists;

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif
		dataFileHour = hour(t);
		make_filename(logFilename, t);

		Serial.println(logFilename);
		exists = sd.exists(logFilename);

		if (dataFile.open(logFilename, O_CREAT | O_WRITE | O_APPEND)) {
			dataFileOpened = 1;
			if (!exists)
				dataFile.println(F("ts,time,count,direction,speed,length,bat_mV"));
		} else {
			Serial.println(F("Cannot open logfile"));
			return 1;
		}
	}

	return 0;
}

int sdRawFileOpen()
{
	if (!rawFileOpened) {
		if (rawLogFile.open("raw.log", O_CREAT | O_WRITE | O_APPEND)) {
			rawFileOpened = 1;
		} else {
			Serial.println(F("Cannot open raw log file"));
			return -EIO;
		}
	}

	return 0;
}

int logToSd(int count, int direction, float speed, float length, time_t time, bool verbose)
{
	static char buf[STRBUF_TIME_SIZE];

	sprintf_time(buf, time);

	if (verbose) {
		Serial.print("V:");
		Serial.print(time - LOCAL_TIME_OFFSET); /* UTC Unix timestamp */
		Serial.print(",");
		Serial.print(buf);
		Serial.print(",");
		Serial.print(count);
		Serial.print(",");
		Serial.print(direction);
		Serial.print(",");
		Serial.print(speed);
		Serial.print(",");
		Serial.print(length);
		Serial.print(",");
		Serial.println(readBattery_mV());
	}

	if (!sdReady) {
		return 1;
	}

	if (sdFileOpen())
		return 1;

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif

	dataFile.print(time - LOCAL_TIME_OFFSET); /* UTC Unix timestamp */
	dataFile.print(",");
	dataFile.print(buf);
	dataFile.print(",");
	dataFile.print(count);
	dataFile.print(",");
	dataFile.print(direction);
	dataFile.print(",");
	dataFile.print(speed);
	dataFile.print(",");
	dataFile.println(readBattery_mV());
	dataFile.sync();

	return 0;
}

int logToSdRaw(time_t time, struct hit *hit_series, uint8_t count)
{
	if (!sdReady)
		return -EIO;

	if (sdRawFileOpen())
		return -EIO;

		rawLogFile.print(localtime2utc(time));
		rawLogFile.print(", ");
	for (int i = 0; i < count; i++) {
		rawLogFile.print(hit_series[i].channel);
		rawLogFile.print(":");
		rawLogFile.print(hit_series[i].time);
		rawLogFile.print(" ");
	}
	rawLogFile.println();
	rawLogFile.sync();

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


