
#include "RGB.h"

#include <Arduino.h>

#include "TWIP.h"


RGB rgb(RGB_RED_PIN, RGB_GRN_PIN, RGB_BLU_PIN);


RGB::RGB(uint8_t r_pin, uint8_t g_pin, uint8_t b_pin)
{
	rgb_pin[0] = r_pin;
	rgb_pin[1] = g_pin;
	rgb_pin[2] = b_pin;
}

void RGB::task(void)
{
    //return;
	switch (mode)
    {   
        case FLASH:
        {
            if ((millis()-last_ms) >= mode_interval) {
                last_ms = millis();
                _setColor((current_color == mode_color ? BLACK : mode_color));
            }
        } break;

       	case FADE:
       	{
            if ((millis()-last_ms) >= mode_interval) {
                last_ms = millis();
                _setColor(mode_color);
            }
       	} break;

        case MANUAL:
        {

        } break;

        default:
        case OFF:
        {
        	if (current_color != BLACK)
        		_setColor(BLACK);
        } break;
	}
}

void RGB::setColor(uint32_t color)
{
    mode = MANUAL;
    _setColor(color);
}

void RGB::_setColor(uint32_t color)
{
	current_color = color;
	_setValue((color & 0xFF0000) >> 16, (color & 0x00FF00) >> 8, (color & 0x0000FF));  
}

void RGB::setValue(uint8_t r, uint8_t g, uint8_t b)
{
    mode = MANUAL;
    _setValue(r, g, b);
}

void RGB::_setValue(uint8_t r, uint8_t g, uint8_t b)
{
	rgb_color[0] = constrain(r, 0, 255);	
	rgb_color[1] = constrain(g, 0, 255);	
	rgb_color[2] = constrain(b, 0, 255);

	analogWrite(rgb_pin[0], rgb_color[0]);
	analogWrite(rgb_pin[1], rgb_color[1]);
	analogWrite(rgb_pin[2], rgb_color[2]);	
}

void RGB::setFlash(uint32_t color, uint16_t interval)
{
	mode = FLASH;
	mode_color = color;
	mode_interval = interval;
}

void RGB::setFade(uint32_t color, uint16_t duration)
{
	mode = FADE;
	mode_color = color;

	mode_interval = duration;
}

void RGB::setMode(int8_t _mode)
{
    mode = _mode;
}





