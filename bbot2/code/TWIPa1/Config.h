
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <inttypes.h>
#include <Arduino.h>

struct cfg_t {
	double Kp;
	double Ki;
	double Kd;
	double min_error;
	double setpoint;
};

extern cfg_t cfg_table;

const cfg_t cfg_defaults = {
	25.0,	// Kp
	3.05,	// Ki
	5.01,	// Kd
	0.5,	// min_error
	0.0		// setpoint
};

//cfg_t cfg_defaults;


class Config : public cfg_t
{
public:
	Config(void);

	void load(void);
	void save(void);
	void update(void);

	void loadDefaults(void);

	uint16_t readEEPROM(uint16_t addr, cfg_t &value);
	uint16_t writeEEPROM(uint16_t addr, const cfg_t &value);
	uint16_t updateEEPROM(uint16_t addr, const cfg_t &value);
};

extern Config config;


#endif
