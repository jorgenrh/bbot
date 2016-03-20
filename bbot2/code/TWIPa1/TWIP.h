	
#ifndef _TWIP_H_
#define _TWIP_H_


#define TWIP_VERSION 	   0.1


// Sensor definitions
#define I2C_SPEED 		400000L 	// Max MPU6050 i2c speed
#define IMU_CALIBRATION   15000		// ms


// RGB definitions
#define RGB_RED_PIN 	 	10
#define RGB_GRN_PIN 	  	 9
#define RGB_BLU_PIN 	 	12


// Motor definitions				// P# used for speed in Motor.h
#define LED_PIN 		 	11		// D6

#define ENABLE_PIN 		 	16  	// F7
#define MOTOR1_STEP_PIN  	17 		// F6
#define MOTOR1_DIR_PIN 	 	18  	// F5
#define MOTOR2_STEP_PIN  	19 		// F4
#define MOTOR2_DIR_PIN 	 	20  	// F1

#define MAX_SPEED 		   500 		// min speed hardcoded to 20 ticks @ 50kHz


// LCD definitions
#define LCD_RS_PIN 		 	13
#define LCD_RW_PIN 		 	14
#define LCD_E_PIN 		 	15
#define LCD_D4_PIN 		 	 4
#define LCD_D5_PIN 		 	 3
#define LCD_D6_PIN 		 	 2
#define LCD_D7_PIN 		 	 1

#define LCD_WIDTH		 	16
#define LCD_HEIGHT		 	 2


#endif
