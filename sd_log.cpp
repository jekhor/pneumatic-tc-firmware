
#include <SPI.h>
#include <SdFat.h>
#include "sd_log.h"
#include "common.h"

#define SPI_SPEED SD_SCK_MHZ(8)
const uint8_t sd_CS = 10;

static bool sdReady = 0;

SdFat sd;
static SdFile dataFile;

static bool dataFileOpened = 0;
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
	p[1] = 'L';
	p[2] = 'O';
	p[3] = 'G';
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

#ifdef DEBUG_MEMORY
		Serial.print(F("freeMemory()="));
		Serial.println(freeMemory());
#endif
		dataFileHour = hour(t);
		make_filename(logFilename, t);

		Serial.println(logFilename);

		if (dataFile.open(logFilename, O_CREAT | O_WRITE | O_APPEND)) {
			dataFileOpened = 1;
			dataFile.println(F("ts,time,count,direction,speed,bat_mV"));
		} else {
			Serial.println(F("Cannot open logfile"));
			return 1;
		}
	}

	return 0;
}

int logToSd(int count, int direction, float speed, time_t time, bool verbose)
{
	static char buf[STRBUF_TIME_SIZE];

	if (speed < 0)
		speed = 0;

//	Serial.print("Speed km/h ");
//	Serial.println(speed);

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


