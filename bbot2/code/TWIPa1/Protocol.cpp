
#include "Protocol.h"

#include <Arduino.h>

Protocol com;

struct sensor_t {
	double angle;
} __attribute__((packed)) sensor_p;

const char *binCmdHeader = "$B>"; // Standard command header
const char *binResHeader = "$B<"; // Standard response header
const char *commandHeader = "$S>"; // Standard command header
const char *responseHeader = "$S<"; // Standard response header


void Protocol::init(serial_obj &ser, uint32_t baud, uint16_t timeout)
{
	serial = ser;
	serial.begin(baud);
	serial.setTimeout(timeout);
}

void Protocol::task(void)
{
	if (Serial.available()) {
		int inByte = Serial.read();
		Serial.print(inByte, BYTE); 
	}
}
