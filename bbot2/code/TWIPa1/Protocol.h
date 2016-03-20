
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <Arduino.h>


typedef usb_serial_class serial_obj; // usb_serial_class for atmega32u 'Serial', HardwareSerial for the rest?

class Protocol {
public:
	void init(serial_obj &ser, uint32_t baud, uint16_t timeout);
	void task(void);

	serial_obj serial;
};


extern Protocol com;

#endif
