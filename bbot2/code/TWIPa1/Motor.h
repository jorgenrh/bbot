
#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <Arduino.h>

typedef enum {
	STOPPED = -1,
	FORWARD,
	BACKWARD
} Direction;

/*
extern "C" {
	void TIMER1_COMPA_vect(void) __attribute__ ((signal));
};
*/
class Motor
{
public:
	//friend void TIMER1_COMPA_vect (void);

	void init(void);
	void task(void);

	void setSpeed(int16_t new_speed);
	void setDirection(Direction new_dir);

	int8_t getDirection(void);
	int16_t getSpeed(void);
	double getPosition(void);

private:
	void calcPosition(void);

	int8_t current_dir;
	int16_t current_speed;
	double current_pos;

	uint32_t stepper_ticks;
	uint32_t stepper_tick_counter;

	int32_t step_counter;
	int32_t rev_counter;
};


extern Motor motor;


#endif
