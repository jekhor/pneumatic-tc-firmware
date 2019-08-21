
#ifndef _SD_LOG_H
#define _SD_LOG_H

#include <Time.h>
#include <SdFat.h>

void setup_sd();
int sdFileOpen();
char dumpSdLog(char *file);
int logToSd(int count, int direction,
	    float speed, float length, time_t time, bool verbose);
int isSdReady(void);
char *currentLogFilename(void);

extern SdFat sd;

#endif /* _SD_LOG_H */
