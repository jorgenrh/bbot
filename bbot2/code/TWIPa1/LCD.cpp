
#include "LCD.h"

#include <Arduino.h>
#include <LiquidCrystalFast.h>

#include "TWIP.h"

LCD lcd(LCD_RS_PIN, LCD_RW_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

void LCD::init(void)
{
	lcd.begin(LCD_WIDTH, LCD_HEIGHT);
	lcd.clear();
}
