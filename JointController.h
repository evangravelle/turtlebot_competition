#include <Adafruit_MotorShield.h>

class JointController{

public:
  JointController(int, int, int, Adafruit_DCMotor*, int=2);

  //Run the PID update
  void update();

  void move_to(int);

  void set_ki(int);

  //Dump trimpot status
  void dump();



private:
  static int _total_joint_count;
  int _my_index;

  //Constraints
  int _min_pose;
  int _max_pose;

  //Error must be greater than this to move
  int _tolerance;

  //Analog pin for position feedback trimpot
  int _analog_pin;

  //Pointer to adafruit motor
  Adafruit_DCMotor* _af_motor;

  //PID always moves to here
  int _set_point;

  //PID constants
  int _Kp = 2;
  int _Ki = 1;

  //integral accumulator
  int _I = 1;
  int _I_threshold = 30;
  int _I_max = 80;

  //PID state
  int _current;
  int _error;

};




