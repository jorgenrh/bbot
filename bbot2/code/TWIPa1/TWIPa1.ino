#include <Streaming.h>

#include "TWIP.h"
#include "RGB.h"
#include "Config.h"
#include "Protocol.h"
#include "Motor.h"
#include "IMU.h"
#include "LCD.h"
#include "PID.h"

uint32_t last_ms = 0;
uint16_t freq = 0;

void printConfig(void)
{
	Serial << F("Config:") << endl;
	Serial << F(" Kp: ") << config.Kp << endl << F(" Ki: ") << config.Ki << endl << F(" Kd: ") << config.Kd << endl;
	Serial << F(" min_error: ") << config.min_error << endl << F(" setpoint: ") << config.setpoint << endl;
}

void updateLCD(void)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd << F("S: ") << motor.getSpeed() << F(" A: ") << imu.getAngle();
	lcd.setCursor(0, 1);
	lcd << motor.getPosition() << F("m ") << freq << F("Hz");
	//lcd << F("P: ") << motor.getPosition() << F("m F: ") << freq;
}

void setup(void)
{
	//delay(2000);

	config.loadDefaults();

	com.init(Serial, 115200, 10);
	lcd.init();
	motor.init();
	imu.init();
	
	rgb.setFlash(0x0F0F0F, 1000); // ready

	Serial << endl << F("TWIP ") << TWIP_VERSION << F(" Ready") << endl << endl;
	printConfig();
}

void loop(void)
{
	if (imu.dataAvailable())
	{
		double angle = imu.getAngle();

		if (abs(angle) >= 30) {
			motor.setSpeed(0);
		}
		else {
			int16_t new_speed = pid.calculate(angle);
			motor.setSpeed(new_speed);
		}
	}

	rgb.task();
	com.task();
	
	updateLCD();

	uint32_t ms = millis();
	freq = (int)(1/((ms-last_ms)*1e-3));
	last_ms = ms;
}



