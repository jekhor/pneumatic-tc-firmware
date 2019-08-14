
#include <Arduino.h>
#include <stdint.h>
#include "timers.h"

simple_timer_t simple_timers[NUM_TIMERS];

void schedule_timer(uint8_t timer_id, unsigned long timeout)
{
	simple_timers[timer_id].scheduled_at = millis();
	simple_timers[timer_id].timeout = timeout;
	simple_timers[timer_id].active = true;
}

void stop_timer(uint8_t timer_id) {
	simple_timers[timer_id].active = false;
}

/* Try to execute timers. Returns true if any timer remains active */
bool try_timers(void) {
	int i;
	unsigned long t = millis();

	for (i = 0; i < NUM_TIMERS; i++) {
		if (simple_timers[i].active
		    && (t - simple_timers[i].scheduled_at >= simple_timers[i].timeout)) {
			simple_timers[i].active = false;
			simple_timers[i].callback();
		}
	}

	for (i = 0; i < NUM_TIMERS; i++)
		if (simple_timers[i].active)
			return true;

	return false;
}


