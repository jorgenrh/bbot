
#include "Config.h"

#include <inttypes.h>
#include <avr/eeprom.h>


#define CONFIG_ADDRESS 0


Config config;


Config::Config(void)
{
	load();
}

void Config::load(void)
{
	readEEPROM(CONFIG_ADDRESS, (*this));
}

void Config::save(void)
{
	writeEEPROM(CONFIG_ADDRESS, (cfg_t)(*this));
}

void Config::update(void)
{
	updateEEPROM(CONFIG_ADDRESS, (cfg_t)(*this));
}

void Config::loadDefaults(void)
{
	(*(cfg_t*)(this)) = cfg_defaults;
	update();
}

uint16_t Config::readEEPROM(uint16_t addr, cfg_t &value) 
{
	eeprom_busy_wait();
	uint8_t *p = (uint8_t*)(void*)&value;
	uint16_t i;
	for (i = 0; i < sizeof(value); i++)
		*p++ = eeprom_read_byte((uint8_t*)addr++);
	return i;
}

uint16_t Config::writeEEPROM(uint16_t addr, const cfg_t &value) 
{
	eeprom_busy_wait();
	const uint8_t *p = (const uint8_t*)(const void*)&value;
	uint16_t i;
	for (i = 0; i < sizeof(value); i++)
		eeprom_write_byte((uint8_t*)addr++, *p++);
	return i;
}

uint16_t Config::updateEEPROM(uint16_t addr, const cfg_t &value) 
{
	eeprom_busy_wait();
	const uint8_t *p = (const uint8_t*)(const void*)&value;
	uint16_t i;
	for (i = 0; i < sizeof(value); i++) {
		if (eeprom_read_byte((uint8_t*)addr) != *p)
			eeprom_write_byte((uint8_t*)addr, *p);
		addr++;
		p++;
	}
	return i;
}
