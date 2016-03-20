// BalanceBot5.ino

#include <Wire.h>
#include <I2Cdev.h>
//#include <MPU6050.h>
#include <JJ_MPU6050_DMP_6Axis.h>
//#include <MPU6050_6Axis_MotionApps20.h>
//#include "EEPROMAnything.h"



#define DEBUG

#define RAD2DEG 57.295779513

#define ITERM_MAX_ERROR 40 // I-term windup constants
#define ITERM_MAX 5000

#define MAX_ACCEL 100


// BOT settings
float Kp = 0.16; 
float Td = 80; //56;
float Kp_speed = 0.065;//0.085;
float Ti_speed = 0.05;//0.02; 

uint8_t fusionGain = 0x20;

float maxTargetAngle = 12;

float angleOffset = -0.1;

Stream &Ser = Serial1;

/*
struct {

	float Kp = 0.16;
	float Td = 56;
	float Kp_speed = 0.085;
	float Ti_speed = 0.02; 

	uint8_t fusionGain = 0x20;

	float maxTargetAngle = 12;

} config;
*/



// MOTOR pins
const int8_t STBY = 7;			// standby
const int8_t PWMA = 10; 		// speed / Motor A
const int8_t AIN1 = 8;		 	// dir A
const int8_t AIN2 = 9; 		// dir A
const int8_t PWMB = 4; 		// speed / Motor B
const int8_t BIN1 = 6; 		// dir B
const int8_t BIN2 = 5; 		// dir B


// MPU6050 variables
MPU6050 mpu;
bool dmpReady = false;		// set true if DMP init was successful
uint8_t mpuIntStatus;		// holds actual interrupt status byte from MPU
uint8_t devStatus;			// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;		// expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;			// count of all bytes currently in FIFO
uint8_t fifoBuffer[18];		// 64 // FIFO storage buffer
Quaternion q;


// BOT variables
const int8_t ledPin = 13;

uint8_t timerCurrent = 0;
uint8_t timerOld = 0;
uint8_t timerDiff = 0;

float botAngle;
float botAngleOld;

int16_t botSpeed;
int16_t botSpeedOld;
float estimatedSpeedFiltered = 0;

int16_t motorSpeed[2];
uint8_t motorDir[2];

float setPointOld = 0;
float pidErrorOld = 0;
float pidErrorOld2 = 0;
float pidErrorSum;

float controlOutput = 0;





void setup() 
{
	Serial.begin(9600);
	Serial1.begin(9600);

	Wire.begin();
	TWBR = 24; // 400kHz I2C rate

	// set pinModes
	pinMode(STBY, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(AIN1, OUTPUT);
	pinMode(AIN2, OUTPUT);
	pinMode(PWMB, OUTPUT);
	pinMode(BIN1, OUTPUT);
	pinMode(BIN2, OUTPUT);
	
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, LOW);

	

	// initialize MPU6050
	Ser.println(F("Setting up MPU6050..."));
	mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
	mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
	mpu.setSleepEnabled(false);

	delay(2000);

	Ser.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Ser.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;
	} 
	else { // ERROR!
		Ser.print(F("DMP Initialization failed (code "));
		Ser.print(devStatus);
		Ser.println(F("):"));
		switch (devStatus) {
			case 1: Ser.println(F("Initial memory load failed")); break; // most common
			case 2: Ser.println(F("DMP configuration updates failed")); break;
			default: Ser.println(F("Unknown error")); 
		}
	}

	// Gyro calibration
	// The robot must be steady during initialization
	// Time to settle things... the bias_from_no_motion algorithm 
	// needs some time to take effect and reset gyro bias.
	Ser.print(F("Calibrating gyro..."));
	delay(15000);
	Ser.println(F("done"));
	digitalWrite(ledPin, HIGH);

	// verify connection
	Ser.println(F("Testing device connections..."));
	Ser.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// Adjust sensor fusion gain
	Ser.println(F("Adjusting DMP sensor fusion gain..."));
	dmpSetSensorFusionAccelGain(fusionGain);

	mpu.resetFIFO();

}


void loop() 
{
	timerCurrent = millis();

	fifoCount = mpu.getFIFOCount();
	if ((fifoCount < 18) || (fifoCount > 18)) { // 64
		if (fifoCount > 18) {
			Ser.println(F("FIFO reset"));
			mpu.resetFIFO();
		}
		return;
	}

	timerDiff = timerCurrent - timerOld;
	timerOld = timerCurrent;

	botAngleOld = botAngle;
	botAngle = dmpGetTheta();

	mpu.resetFIFO(); // again??

	botSpeedOld = botSpeed;
	botSpeed = (motorSpeed[1] - motorSpeed[2])/2;

	int16_t angularVelocity = (botAngle - botAngleOld)*90;
	int16_t estimatedSpeed = botSpeedOld - angularVelocity;

	estimatedSpeedFiltered = estimatedSpeedFiltered*0.95 + (float)estimatedSpeed*0.05;

	int throttle = 0; // for now (no remote ctrl)
	float targetAngle = speedPIControl(timerDiff, estimatedSpeedFiltered, throttle, Kp_speed, Ti_speed);
	targetAngle = constrain(targetAngle, -maxTargetAngle, maxTargetAngle);

	controlOutput += stabilityPDControl(timerDiff, botAngle, targetAngle, Kp, Td);
	controlOutput = constrain(controlOutput, -255, 255);

	// implement steering here, motor = controlOutput + steering, constrain

	int motor1;
	int motor2;

	if ((botAngle < 40) && (botAngle > -40)) {
		motor1 = controlOutput;
		motor2 = -controlOutput;
	}
	else {
		motor1 = 0;
		motor2 = 0;
	}

	setMotorSpeed(0, motor1);
	setMotorSpeed(1, motor2);

	serialCmd();
}



// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
	// INV_KEY_0_96
	mpu.setMemoryBank(0);
	mpu.setMemoryStartAddress(0x60);
	mpu.writeMemoryByte(0);
	mpu.writeMemoryByte(gain);
	mpu.writeMemoryByte(0);
	mpu.writeMemoryByte(0);
}

float dmpGetTheta() 
{
	mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
	mpu.dmpGetQuaternion(&q, fifoBuffer); 
	mpu.resetFIFO();  // We always reset FIFO

	return (atan2(-2*(q.x*q.z - q.w*q.y), 1-2*(q.x*q.x + q.y*q.y)) * RAD2DEG) + angleOffset;
}



// PID FUNCTIONS
// Cascade PID, PD for stability -> PI for speed control, DT in milliseconds
//
// PD implementation
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Td)
{
	float error;
	float output;

	error = setPoint-input;

	// Td is implemented in two parts
	//    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
	//    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
	output = Kp*error + (Td*(setPoint - setPointOld) - Td*(input - pidErrorOld2))/DT; // + error - PID_error_Old2
	//Ser.print(Kd*(error-PID_errorOld));Ser.print("\t");
	pidErrorOld2 = pidErrorOld;
	pidErrorOld = input;  // error for Kd is only the input component
	setPointOld = setPoint;
	
	return output;
}

// PI implementation
float speedPIControl(float DT, float input, float setPoint, float Kp, float Ti)
{
	float error;
	float output;

	error = setPoint - input;
	pidErrorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
	pidErrorSum = constrain(pidErrorSum, -ITERM_MAX, ITERM_MAX);

	output = Kp*error + Ti*pidErrorSum*DT*0.001;
	
	return output;
}



// MOTOR FUNCTIONS
//
// 
void setMotorSpeed(int motor, int speed)
{
	digitalWrite(STBY, HIGH); //disable standby

	if ((motorSpeed[motor] - speed) > MAX_ACCEL)
		motorSpeed[motor] -= MAX_ACCEL;
  	else if ((motorSpeed[motor] - speed) < -MAX_ACCEL)
		motorSpeed[motor] += MAX_ACCEL;
	else
		motorSpeed[motor] = speed;

	bool dir = (speed < 0 ? HIGH : LOW);

	if (motor == 1) {
		digitalWrite(AIN1, dir);
		digitalWrite(AIN2, !dir);
		analogWrite(PWMA, abs(speed));
	}
	else {
		digitalWrite(BIN1, !dir);
		digitalWrite(BIN2, dir);
		analogWrite(PWMB, abs(speed));
	}
}







