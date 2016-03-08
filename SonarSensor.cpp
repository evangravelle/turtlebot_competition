#include "Arduino.h"
#include "SonarSensor.h"

long* SonarSensor::_samples = new long[NUM_PINGS];

SonarSensor::SonarSensor(int trig, int echo){
	_trig_pin = trig;
	_echo_pin = echo;
	pinMode(_trig_pin, OUTPUT);
	pinMode(_echo_pin, INPUT);
}

int SonarSensor::get_distance(){
	return _distance;
}

long SonarSensor::update(){
	long s1 = ping();
	delay(5);
	long s2 = ping();
	delay(5);
	long diff = abs(s1 - s2) / 2 / 29;
	if(diff < 5){
		_distance = (s1 + s2) / 2 / 29 / 2;
	}else{
		_distance = 0;
	}
	return diff;
}

long SonarSensor::update_averaging(){
	long avg = 0;
	long sample;
	for(int i=0;i<NUM_PINGS;i++){
		sample = ping();
		delay(5);
		if(sample == 0){
			_distance = 0;
			return 0;
		}
		avg += sample;
		_samples[i] = sample;
	}

	avg /= NUM_PINGS;
	long var=0;
	long diff;
	for(int i=0;i<NUM_PINGS;i++){
		diff = _samples[i] - avg;
		var += diff * diff;
	}
	var /= NUM_PINGS;
	var /= (58*58);
	if(var <= 10){
		_distance = avg / 29 / 2;
	}else{
		_distance = 0;
	}
	return var;
}

long SonarSensor::ping(){
	digitalWrite(_trig_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_trig_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_trig_pin, LOW);

	return pulseIn(_echo_pin, HIGH, 6000);
	
	// return duration / 29 / 2;
}

