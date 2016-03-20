
#ifndef _LCD_H_
#define _LCD_H_


#include <Arduino.h>
#include <LiquidCrystalFast.h>


class LCD : public LiquidCrystalFast
{
public:
	LCD(uint8_t rs, 
		uint8_t rw, 
		uint8_t e,
		uint8_t d4, 
		uint8_t d5, 
		uint8_t d6, 
		uint8_t d7) : LiquidCrystalFast(rs, 
										rw, 
										e, 
										d4, 
										d5, 
										d6, 
										d7)
										{}

	void init(void);
};


extern LCD lcd;

#endif
