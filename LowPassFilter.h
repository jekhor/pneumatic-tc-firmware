#ifndef __LOWPASSFILTER_H__
#define __LOWPASSFILTER_H__

#include <Arduino.h>

class LowPassFilter {
#define LP_Na	8191
#define LP_Nb	1
#define LP_K	13
#define LP_Order 20

	public:
	LowPassFilter(unsigned long int initY=0);

	void input(unsigned long int x);
	unsigned long int output();
	void setOutput(unsigned long int y);

	private:
	unsigned long long int Y;
};

#endif
