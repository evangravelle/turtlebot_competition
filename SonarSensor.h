#include "Arduino.h"

class SonarSensor{
public:
	SonarSensor(int, int);

	int get_distance();

private:
	int _trig_pin;
	int _echo_pin;

};



