#include "Arduino.h"
#include "SonarSensor.h"


SonarSensor::SonarSensor(int trig, int echo){
	_trig_pin = trig;
	_echo_pin = echo;
	pinMode(_trig_pin, OUTPUT);
	pinMode(_echo_pin, INPUT);
}

int SonarSensor::get_distance(){
	digitalWrite(_trig_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_trig_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_trig_pin, LOW);

	long duration = pulseIn(_echo_pin, HIGH, 30000);
	
	return duration / 29 / 2;
}

