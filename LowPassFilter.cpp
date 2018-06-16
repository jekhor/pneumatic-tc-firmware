#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(unsigned long int initY) {
	Y = (unsigned long long)initY << LP_Order;
}

void LowPassFilter::setOutput(unsigned long int y) {
	Y = (unsigned long long)y << LP_Order;
}

void LowPassFilter::input(unsigned long int x) {
	Y = (LP_Na * Y + LP_Nb * ((unsigned long long)x << LP_Order)) >> LP_K;
}

unsigned long int LowPassFilter::output() {
	return Y >> LP_Order;
}


