
#include "PID.h"
#include <Streaming.h>
#include <Arduino.h>

#include "TWIP.h"
#include "Config.h"
#include "Motor.h"


PID pid;


PID::PID(void)
{
	pre_error = 0;
	integral = 0;
	last_micros = micros();
}

int16_t PID::calculate(double actual_position)
{
	double error;
	double derivative;
	int output;

	uint32_t us = micros();
	uint32_t dt = us - last_micros;
	last_micros = us;
	
	//Caculate P,I,D
	error = config.setpoint - actual_position;
	//error = 0 - actual_position;

	//In case of error too small then stop integration 
	//if (abs(error) > 0.01) {
	if (abs(error) > config.min_error) {
		integral = integral + error*dt*1e-6; 
	}
	
	derivative = (error - pre_error)/(dt*1e-6);
	
	//Serial << "Kp: " << config.Kp << "Ki: " << config.Ki << "Kd: " << config.Kd << endl;

	output = (int)(config.Kp*error + config.Ki*integral + config.Kd*derivative);

	output = constrain(output, -MAX_SPEED, MAX_SPEED);

	//Update error 
	pre_error = error;

	//output = -20;

	return output; 
}

