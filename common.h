#ifndef _COMMON_H
#define _COMMON_H

#include <Time.h>

#define IDLE_TIMEOUT_TIMER	0
#define IDLE_TIMEOUT_MS		60000

#define LED_TIMER		1
#define	LED_FLASH_ON_MS		10
#define LED_FLASH_OFF_MS	200

#define LOCAL_TIME_OFFSET (3 * SECS_PER_HOUR)

#define localtime2utc(time) ((time) - LOCAL_TIME_OFFSET)

#define BT_EN_POWEROFF_TIMEOUT_MS 10000

#define ARRAY_SIZE(_a)	(sizeof(_a) / sizeof((_a)[0]))

#define NUM_CHANNELS	2
#define THRESHOLD	25
#define HYSTERESIS	6
#define MAX_HITS	32

#define LED_PIN			4
#define PRESSURE_SENS_EN_PIN	3
#define PWR_ON_PIN		8
#define BT_EN_PIN		17
#define BT_STATE_PIN		9
#define BT_START_PIN		2
#define BATT_MEASURE_PIN	A6

enum bt_mode {
	BT_AUTO,
	BT_PERMANENT,
};

struct hit {
	uint8_t channel;
	uint16_t time;
};


// average spacing between wheels of bicycle in meters
const float wheel_spacing = 1.06;
/* 25 cm between two parallel sensors */
const float sensors_spacing = 0.5;

extern bool raw_measuring;
extern enum bt_mode bluetooth_mode;
extern volatile bool bt_enabled;
extern uint8_t idle_sleep_mode;

unsigned short readBattery_mV(); 

char *itoa_10lz(char *buf, uint16_t val, uint8_t digits);
#define STRBUF_TIME_SIZE	20
char *sprintf_time(char *buf, time_t time);
void printTime(time_t time);
void set_bluetooth_mode(enum bt_mode mode);

#endif /* _COMMON_H */
