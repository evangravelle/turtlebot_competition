

#include "JointController.h"

// #define PID_VERBOSE
// #define ACK_MOVEMENTS

int JointController::_total_joint_count = 0;

JointController::JointController( int min, 
                                  int max, 
                                  int analogPin, 
                                  Adafruit_DCMotor* motor,
                                  int tolerance){
  _my_index = _total_joint_count++;

  _min_pose = min;
  _max_pose = max;
  _tolerance = tolerance;
  _analog_pin = analogPin;
  if(!motor)
    Serial.println("adafruit motor library is fucked");
  
  _af_motor = motor;
  _set_point = (min + max) / 2;
}


int JointController::update(){
  int speed;
  int set_ok = 0;

  _current = analogRead(_analog_pin);
  _error = _set_point - _current;

  int abs_error = abs(_error);

  if(abs_error <= _I_threshold){
    _I = min(_I + _Ki * abs_error, _I_max);
  }

  //decide whether to move at all
  if(abs_error > _tolerance){
    speed = constrain(_Kp*abs_error + _I, 50, 255);
  }else{
    set_ok = 1;
    _I = 0;
    speed = 0;
    _set_point = _current;
  }
  
#ifdef PID_VERBOSE
  Serial.print(_my_index);
  Serial.print("    current: ");
  Serial.print(_current);
  Serial.print("    set_point: ");
  Serial.print(_set_point);
  Serial.print("    error: ");
  Serial.print(_error);
  Serial.print("    I: ");
  Serial.println(_I);
  // delay(10);
#endif
  
  //decide if forward or backward
  if (_error > 0) {
    _af_motor->run(FORWARD);
    _af_motor->setSpeed(speed);
  } else {
    _af_motor->run(BACKWARD);
    _af_motor->setSpeed(speed);
  }

  return set_ok;
}

void JointController::move_to(int new_set_point){
  if(new_set_point >= _min_pose && new_set_point <= _max_pose){
    _set_point = new_set_point;
    
#ifdef ACK_MOVEMENTS
    Serial.print("move ");
    Serial.print(_my_index);
    Serial.print(" to ");
    Serial.println(_set_point);
#endif

  }
}

void JointController::set_ki(int ki){
  if(ki >= 0 && ki < 20)
    _Ki = ki;
}

void JointController::dump(){
  Serial.print(analogRead(_analog_pin));
}



