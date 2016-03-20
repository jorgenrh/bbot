

#ifndef _RGB_H_
#define _RGB_H_

#include <Arduino.h>

class RGB
{
public:
	RGB(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin);

	void task(void);
	void setColor(uint32_t color);
	void setValue(uint8_t r, uint8_t g, uint8_t b);

	void setFlash(uint32_t color, uint16_t interval);
	void setFade(uint32_t color, uint16_t duration);

	void setMode(int8_t _mode);

	enum Mode {
		FLASH,
		FADE,
		MANUAL,
		OFF
	};

	enum Color {
		BLACK = 0x000000,
		RED = 0xFF0000,
		GREEN = 0x00FF00,
		BLUE = 0x0000FF
	};

private:
	void _setColor(uint32_t color);
	void _setValue(uint8_t r, uint8_t g, uint8_t b);

	uint32_t last_ms;

	uint8_t rgb_pin[3];
	uint8_t rgb_color[3];
	uint32_t current_color;

	uint8_t mode;
	uint32_t mode_color;
	uint16_t mode_interval;
};


extern RGB rgb;


#endif
