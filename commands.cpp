
#include <Arduino.h>
#include <Time.h>
#include <DS1307RTC.h>
#include <SdFat.h>

#include "common.h"
#include "sd_log.h"
#include "timers.h"



static int scan_time(char *date, char *time,
		unsigned short *y, tmElements_t *tm)
{
	uint16_t numbers[3];
	char *c;
	int i = 0;

	c = strtok(date, "-");
	while (c && (i < 3)) {
		numbers[i] = atoi(c);
		c = strtok(NULL, "-");
		i++;
	}

	if (i != 3)
		return 1;

	*y = numbers[0];
	tm->Month = numbers[1];
	tm->Day = numbers[2];

	i = 0;
	c = strtok(time, ":");
	while (c && (i < 3)) {
		numbers[i] = atoi(c);
		c = strtok(NULL, ":");
		i++;
	}

	if (i != 3)
		return 1;

	tm->Hour = numbers[0];
	tm->Minute = numbers[1];
	tm->Second = numbers[2];

	return 0;
}

static char cmd_time(char argc, char *argv[]) {
	unsigned short int y;
	tmElements_t tm;

	if (argc != 2) {
		Serial.println(F("Usage: time YYYY-MM-DD HH:MM:SS"));
		return 1;
	}

	if (scan_time(argv[0], argv[1], &y, &tm)) {
		Serial.println(F("Cannot parse time"));
		return 1;
	}

	tm.Year = y - 1970;
	if (!RTC.write(tm))
		Serial.println(F("RTC error"));

	setTime(makeTime(tm));
	if (!RTC.read(tm))
		Serial.println(F("RTC error"));

	printTime(now());

	return 0;
}

static char cmd_ls(char, char *[]) {
	if (!isSdReady())
		return 1;

	sd.ls(LS_R | LS_SIZE);

	return 0;
}

static SdFile root;
static SdFile file;
static char filename[13];

static char cmd_dumpall(char, char *[])
{
	if (!isSdReady())
		return 1;

	if (!root.open("/")) {
		Serial.println(F("Root open failed"));
		return 1;
	}

	while (file.openNext(&root, O_RDONLY)) {
		if (file.isDir())
			goto next_file;


		if (!file.getName(filename, sizeof(filename)))
			goto next_file;

		if (!strstr(filename, ".CSV"))
			goto next_file;

		Serial.println(filename);
		dumpSdLog(filename);

next_file:
		file.close();
	}

	root.close();
	return 0;
}

static char cmd_dump(char argc, char *argv[]) {
	if (argc == 1)
		return dumpSdLog(argv[0]);
	else
		return dumpSdLog(NULL);

	return 0;
}

static char cmd_raw(char, char *[]) {
	raw_measuring = !raw_measuring;

	if (raw_measuring) {
		if (raw_measuring)
			stop_timer(IDLE_TIMEOUT_TIMER);
		else
			schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);
	}

	return 0;
}

static char cmd_batt(char, char *[]) {
	Serial.println(readBattery_mV());

	return 0;
}

static char cmd_poff(char, char *[]) {
	digitalWrite(LED_PIN, 1);
	digitalWrite(PWR_ON_PIN, 0);
	return 0;
}

static char cmd_bt(char argc, char *argv[]) {
	if (argc != 1) {
		Serial.print(F("Bluetooth "));
		if (bluetooth_mode == BT_PERMANENT)
			Serial.println(F("ON"));
		else
			Serial.println(F("AUTO"));
	} else {
		if (!strcmp(argv[0], "on")) {
			set_bluetooth_mode(BT_PERMANENT);
		} else if (!strcmp(argv[0], "auto")) {
			set_bluetooth_mode(BT_AUTO);
		}
	}
	return 0;
}

static char cmd_help(char, char *[]) {
	puts_P(PSTR(
	"Commands:\n"
	" time [yyyy-mm-dd HH:MM:SS] \t - get/set time\n"
	" raw\t\t\t\t - toggle raw dump\n"
	" ls\t\t\t\t - list files on SD\n"
	" dump [file]\t\t\t - dump file content\n"
	" dumpall [file]\t\t\t - dump all logs content\n"
	" batt\t\t\t\t - show battery voltage\n"
	" bt [on|auto]\t\t\t - bluetooth enabled permanently/on demand (by magnet)\n"
	));

	return 0;
}

typedef char (*cmd_handler_t)(char argc, char *argv[]);

struct cmd_table_entry {
	PGM_P cmd;
	cmd_handler_t handler;
};

static const char cmd_time_name[] PROGMEM = "time";
static const char cmd_ls_name[] PROGMEM = "ls";
static const char cmd_dump_name[] PROGMEM = "dump";
static const char cmd_dumpall_name[] PROGMEM = "dumpall";
static const char cmd_raw_name[] PROGMEM = "raw";
static const char cmd_poff_name[] PROGMEM = "poff";
static const char cmd_batt_name[] PROGMEM = "batt";
static const char cmd_bt_name[] PROGMEM = "bt";
static const char cmd_help_name[] PROGMEM = "help";

#define CMD(name) {cmd_ ## name ## _name, cmd_ ## name}

static PROGMEM const struct cmd_table_entry cmd_table[] = {
	CMD(time),
	CMD(ls),
	CMD(dump),
	CMD(dumpall),
	CMD(raw),
	CMD(poff),
	CMD(batt),
	CMD(bt),
	CMD(help),
};

#define MAX_CMD_ARGS	3

void parse_cmdline(char *buf, uint8_t) {
	char *cmd = strtok(buf, " ");
	struct cmd_table_entry cmd_entry;
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
		memcpy_P(&cmd_entry, &cmd_table[i], sizeof(cmd_entry));
		if (!strcmp_P(cmd, cmd_entry.cmd)) {
			ret = cmd_entry.handler(argc, argv);
			break;
		}
	}

	if (ret)
		Serial.println("ERROR");

}

#define SERIAL_BUF_LEN	60

static char serial_buf[SERIAL_BUF_LEN];
static uint8_t serial_buf_level = 0;

void processSerial() {
	char c;

	while (Serial.available()) {

		idle_sleep_mode = SLEEP_MODE_IDLE;
//		digitalWrite(LED_PIN, 1);
		schedule_timer(IDLE_TIMEOUT_TIMER, IDLE_TIMEOUT_MS);

		c = Serial.read();
		Serial.write(c);

//		digitalWrite(LED_PIN, 1);
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
//		digitalWrite(LED_PIN, 0);
}


