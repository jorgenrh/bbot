
#include "IMU.h"

#include <Arduino.h>
#include <Streaming.h>
#include <I2Cdev.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <JJ_MPU6050_DMP_6Axis.h>
#include <Wire.h>

#include "TWIP.h"
#include "RGB.h"
#include "LCD.h"
#include "PID.h"
#include "Motor.h"

MPU6050 _mpu;
IMU imu(_mpu);

IMU::IMU(MPU6050 &_mpu)
{
	dmp_ready = false;
	angle = 0;
	last_ms = 0;

	mpu = _mpu;
}

void IMU::init(void)
{
	Serial.print(F("Initializing I2C devices...."));

	Wire.begin();
	TWSR = 0;
	TWBR = ((16000000L/I2C_SPEED)-16)/2;
	TWCR = 1<<TWEN;
  
	//mpu.initialize();
	mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
	mpu.setRate(0);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
	mpu.setSleepEnabled(false);
	delay(100);
	Serial.println(F("done"));	

	Serial.print(F("Initializing DMP............"));
	dev_status = mpu.dmpInitialize();
	Serial.println("done");	
	if (dev_status == 0) 
	{
		// turn on the DMP, now that it's ready
		Serial.print(F("Enabling DMP................"));
		mpu.setDMPEnabled(true);
		mpu_int_status = mpu.getIntStatus();
		dmp_ready = true; 
		Serial.println(F("done"));	
	} 
	else 
	{ 
		Serial << F("DMP Initialization failed (code ") << dev_status << F("): ");
		switch (dev_status) {
		    case 1: Serial << F("Initial memory load failed"); break;
		    case 2: Serial << F("DMP configuration updates failed"); break;
		    default: Serial << F("Unknown error");
		}
		rgb.setFlash(rgb.RED, 200);
		while (1) { rgb.task(); } // Stop program
    }

    calibrate();
}


void IMU::updateAngle(void) 
{
	mpu.getFIFOBytes(fifo_buffer, 16); 		// Read only Quaternion
	mpu.dmpGetQuaternion(&q, fifo_buffer); 
	mpu.resetFIFO();						// Reset FIFO after read

	angle = (asin(-2*(q.x * q.z - q.w * q.y)) * RAD_TO_DEG); // Convert to roll-angle in degrees
}

// Calibration of dmp/IMU, ~10 sec
void IMU::calibrate(void)
{
	lcd.clear();
	lcd.setCursor(0, 1);
	lcd.print(F("Calibrating IMU"));
	
	Serial.print(F("Calibrating IMU......"));
	
	rgb.setFlash(0xFF2100, 25);	

	last_ms = millis();
	while ((millis() - last_ms) < IMU_CALIBRATION) { rgb.task(); };	
	
	Serial.println(F(".......done"));
	
	lcd.clear();
	rgb.setColor(0x000000);
}

bool IMU::dataAvailable(void)
{
	fifo_count = mpu.getFIFOCount();
	if (fifo_count >= 18)
	{
		if (fifo_count > 18) {
			mpu.resetFIFO();
			return false;
		}

		updateAngle();

		return true;
	}

	return false;
}

double IMU::getAngle(void)
{
	return angle;
}




