

enum {
	setKp = 'P',
	setKd = 'D',
	setKpSpeed = 'p',
	setKiSpeed = 'i',
	getPID = 'S',
	getData	= 'Q'
};

//float Kp, Kd, Kp_speed, Ki_speed;

// COM FUNCTIONS
//
void serialCmd()
{
	char cmd;
	if (Ser.available() > 0) {
		cmd = Ser.read();
	} else {
		return;
	}

	switch (cmd) {
		
		case 'P': Kp = Ser.parseFloat(); cmd = 'S'; break;
		case 'D': Td = Ser.parseFloat(); cmd = 'S'; break;
		case 'p': Kp_speed = Ser.parseFloat(); cmd = 'S'; break;
		case 'i': Ti_speed = Ser.parseFloat(); cmd = 'S'; break;
		case 'o': angleOffset = Ser.parseFloat(); cmd = 'S'; break;

	}

	switch (cmd) {

		case 'H': 
		case 'h':
			Ser.println(F("\nP<float> - set Kp"));
			Ser.println(F("D<float> - set Td"));
			Ser.println(F("p<float> - set Kp_speed"));
			Ser.println(F("i<float> - set Ti_speed"));
			Ser.println(F("o<float> - set angleOffset\n"));
			Ser.println(F("S - read settings"));
			Ser.println(F("Q - read angle and motor speed\n"));
			break;

		case 'S': 
		case 's': 
			Ser.print(F("Kp = ")); Ser.println(Kp);
			Ser.print(F("Td = ")); Ser.println(Td);
			Ser.print(F("Kp_speed = ")); Ser.println(Kp_speed);
			Ser.print(F("Ti_speed = ")); Ser.println(Ti_speed);
			Ser.print(F("angleOffset = ")); Ser.println(angleOffset);
			break;

		case 'q':
			Ser.print(F("Angle: ")); Ser.println(botAngle);
			Ser.print(F("Speed: ")); Ser.println(controlOutput);
			break;

		case 'Q':
			Ser.print("Q;"); Ser.print(botAngle);
			Ser.print(";"); Ser.println(controlOutput);
			break;

		//default:
			// do nothing
	}
}

