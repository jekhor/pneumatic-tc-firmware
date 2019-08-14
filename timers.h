#ifndef _TIMERS_H
#define _TIMERS_H

#define NUM_TIMERS		2

struct simple_timer {
	bool active;
	unsigned long scheduled_at;
	unsigned long timeout;
	void (*callback)(void);
};

typedef struct simple_timer simple_timer_t;

extern simple_timer_t simple_timers[NUM_TIMERS];

void schedule_timer(uint8_t timer_id, unsigned long timeout);
void stop_timer(uint8_t timer_id);
bool try_timers(void);


#endif /* _TIMERS_H */
