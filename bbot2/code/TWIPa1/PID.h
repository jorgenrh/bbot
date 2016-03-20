
#ifndef _PID_H_
#define _PID_H_

#include <Arduino.h>


class PID
{
public:
	PID(void);
	
	int16_t calculate(double actual_position);

private:
	double pre_error;
	double integral;
	uint32_t last_micros;
};

extern PID pid;


#endif
