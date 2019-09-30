
#ifndef _SD_LOG_H
#define _SD_LOG_H

#include <Time.h>
#include <SdFat.h>

extern const char sd_counter_dir[];

void setup_sd();
int sdFileOpen();
char dumpSdLog(char *file);
int logToSd(int count, int direction,
	    float speed, float length, time_t time, bool verbose);
int logToSdRaw(time_t time, struct hit *hit_series, uint8_t count);
int isSdReady(void);
char *currentLogFilename(void);

extern SdFat sd;
extern SdFile root;

#endif /* _SD_LOG_H */
