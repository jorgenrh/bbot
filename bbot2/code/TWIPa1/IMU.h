
#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
//#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>
#include <JJ_MPU6050_DMP_6Axis.h>

class IMU
{
public:
	IMU(MPU6050 &_mpu);
	//IMU();

	void init(void);
	
	bool dataAvailable(void);
	void updateAngle(void);
	void calibrate(void);
	double getAngle(void);
	
protected:

	MPU6050 mpu;

	bool dmp_ready;  // set true if DMP init was successful
	uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU (not really needed?)
	uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
	uint16_t fifo_count;     // count of all bytes currently in FIFO
	uint8_t fifo_buffer[18]; // FIFO storage buffer
	Quaternion q;

	double angle;

	uint32_t last_ms;
};

extern IMU imu;



#endif
