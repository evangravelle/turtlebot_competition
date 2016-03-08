#include "Arduino.h"

#define NUM_PINGS 8

class SonarSensor{
public:
	SonarSensor(int, int);

	//Get the distance in cm
	//Return 0 if nothing detected
	int get_distance();

	//Update the calculated distance
	long update_averaging();

	long update();

	//RE VERIFY OUR RANGE TO TARGET
	// but captain, they'll...
	// One ping, please?
	long ping();

private:
	//The last calculated distance as measured in update()
	static long *_samples;

	int _distance;

	int _trig_pin;
	int _echo_pin;

};

