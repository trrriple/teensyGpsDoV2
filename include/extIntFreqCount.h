#ifndef ExtIntFreqCount_h
#define ExtIntFreqCount_h

#include <Arduino.h>

class ExtIntFreqCountClass {
public:
	static void begin(int checkPin, int pulseDir);
	static uint8_t available(void);
	static uint32_t read(void);
	static void end(void);
};

extern ExtIntFreqCountClass ExtIntFreqCount;

#endif
