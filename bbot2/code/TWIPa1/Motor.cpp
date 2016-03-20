
#include "Motor.h"

#include <Arduino.h>

#include "TWIP.h"


#define setbit(p, b) (p |= (1 << b))
#define clrbit(p, b) (p &= ~(1 << b))

Motor motor;

void Motor::init(void)
{
	pinMode(LED_PIN, OUTPUT);
	pinMode(ENABLE_PIN, OUTPUT);
	pinMode(MOTOR1_STEP_PIN, OUTPUT);
	pinMode(MOTOR2_STEP_PIN, OUTPUT);
	pinMode(MOTOR1_DIR_PIN, OUTPUT);
	pinMode(MOTOR2_DIR_PIN, OUTPUT);
	  
	digitalWrite(ENABLE_PIN, LOW);
	digitalWrite(MOTOR1_STEP_PIN, LOW);
	digitalWrite(MOTOR2_STEP_PIN, LOW);
	digitalWrite(MOTOR1_DIR_PIN, LOW);
	digitalWrite(MOTOR2_DIR_PIN, LOW);

	// Overwrite Timer1 used by Wire to use with stepper pulses

	// Timer1 CTC mode
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11); 
	TCCR1A &= ~(1<<WGM10);

	// Output mode = 00 (disconnected)
	TCCR1A &= ~(3<<COM1A0); 
	TCCR1A &= ~(3<<COM1B0); 

	// Set the timer prescaler; divider of 8, resulting in a 2MHz timer on 16MHz
	TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

	// f = CPU / (Prescaler * (OCR1A + 1)) 
	//OCR1A = 125;  //  16 kHz
	//OCR1A = 100;  //  20 kHz
	//OCR1A = 80;   //  25 kHz
	OCR1A = 40;		//  50 kHz
	//OCR1A = 20; 	// 100 kHz
	//OCR1A = 2; 	//   1 MHz
	TCNT1 = 0;

	TIMSK1 |= (1<<OCIE1A);  // Enable Timer1

	setSpeed(0);
}


ISR(TIMER1_COMPA_vect)
//void TIMER1_COMPA_vect(void)
{
	motor.task();
}

void Motor::task(void)
{
	if (current_dir == STOPPED)
		return;

	stepper_tick_counter++;

	if (stepper_tick_counter >= stepper_ticks)
	{
		stepper_tick_counter = 0;

		setbit(PORTF, 6); // motor 1
		setbit(PORTF, 4); // motor 2
		delayMicroseconds(1); // min pulse period
		clrbit(PORTF, 6);
		clrbit(PORTF, 4);

		if (current_dir == FORWARD)
			step_counter++;
		else
			step_counter--;

		calcPosition();
	}
}

void Motor::calcPosition(void)
{
	// @todo ...tmp...
	//if ((abs(step_counter) % 1600) == 0) {
	if (abs(step_counter) >= 1600) {
		if (step_counter < 0) rev_counter--;
		else rev_counter++;
		step_counter = 0;
		setbit(PORTD, 6); // LED
	} else if (abs(step_counter) >= 20) {
		clrbit(PORTD, 6);
	}

	int32_t tot_steps = rev_counter*1600 + step_counter;
	current_pos = ((M_PI*0.1)/1600)*tot_steps;
}


void Motor::setDirection(Direction new_dir)
{
	switch (new_dir) 
	{
		case BACKWARD:
		{
			clrbit(PORTF, 7); // enable
			setbit(PORTF, 5); // motor 1 
			clrbit(PORTF, 1); // motor 2		
		} break;

		case FORWARD:
		{
			clrbit(PORTF, 7); // enable
			clrbit(PORTF, 5); // motor 1
			setbit(PORTF, 1); // motor 2
		} break;

		default:
		case STOPPED:
		{
			setbit(PORTF, 7); // disable
		} break;
	}

	current_dir = new_dir;
}

void Motor::setSpeed(int16_t new_speed)
{
	new_speed = constrain(new_speed, -MAX_SPEED, MAX_SPEED);

	if (new_speed == 0)
		setDirection(STOPPED);
	else if (new_speed < 0) 
		setDirection(BACKWARD);
	else
		setDirection(FORWARD);

	current_speed = new_speed; 
	stepper_ticks = (MAX_SPEED - abs(current_speed)) + 20; // min 20 ticks
}

int8_t Motor::getDirection(void)
{
	return current_dir;
}

int16_t Motor::getSpeed(void)
{
	return current_speed;
}

double Motor::getPosition(void)
{
	return current_pos;
}











