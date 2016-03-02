
/*

This controls the arm 
Input format over serial: <arm index> <set point>|
e.g. 
0 200|1 400|2 300|3 400

*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <stdint.h>
// #include "utility/Adafruit_MS_PWMServoDriver.h"
#include "JointController.h"
#include "CommandQueue.h"
#include "SonarSensor.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


// Select which 'port' M1, M2, M3 or M4. In this case, M1
JointController joint_0(80, 700, A0, AFMS.getMotor(1));
JointController joint_1(94, 797, A1, AFMS.getMotor(2));
JointController joint_2(275, 670, A2, AFMS.getMotor(3));
JointController joint_3(280, 640, A3, AFMS.getMotor(4), 10);

#define NUM_MOTORS 4
JointController* joints[] = {&joint_0, &joint_1, &joint_2, &joint_3};

#define BUF_SIZE 16
char buffer[BUF_SIZE];

SonarSensor sensors[] = {
  SonarSensor(13,12),   //FRONT
  SonarSensor(10,11),   //RIGHT
  SonarSensor(9,8)      //LEFT
  // SonarSensor(7,6), 
  // SonarSensor(4,5)
};

#define NUM_SONAR (sizeof(sensors) / sizeof(SonarSensor))

CommandQueue cmd_queue;

int readUntilPipe(char *buffer, int buf_size){
  long start = millis();
  int pos = 0;
  while(millis() - start < 200 && pos < buf_size){
    if(Serial.available()){
      buffer[pos] = Serial.read();
      if(buffer[pos]=='|')
        return pos;
      pos++;
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(200);
  Serial.print("hello");
  AFMS.begin();  // create with the default frequency 1.6KHz

  // joint_3.move_to(240);
  joint_3.set_ki(0);
}

int select;
int set_point = 500;
int i;

uint8_t waiting_mode = 0;

JointCmd current_cmd;

void loop() {
  
  // delay(500);
  // Serial.println("hello from the other side");
  if (Serial.available()) {
    int read = readUntilPipe(buffer, BUF_SIZE);
    if(!read)
      return;

    if(buffer[0]=='a' || buffer[0]=='A'){
      //Dump analog readings from trimpots
      for(i=0; i<NUM_MOTORS; i++){
        Serial.print(i);
        Serial.print(" ");
        joints[i]->dump();
        Serial.print("|");
      }
    }else if(buffer[0]=='w' || buffer[0]=='W'){
      //Add a wait command to the queue
      current_cmd.select=-1;
      current_cmd.set_point=-1;
      current_cmd.wait = 1;
      cmd_queue.enq(current_cmd);
    }else if(buffer[0]=='s' || buffer[0]=='S'){
      //Get readings from sonar sensors
      for(int i=0;i<NUM_SONAR;i++){
        Serial.print("s");
        Serial.print(i);
        Serial.print(" ");
        Serial.print(sensors[i].get_distance());
        Serial.print("|");
      }
    }else{
      //enqueue motor commands
      select = atoi(buffer);
      char * next = strchr(buffer, ' ');
      set_point = 0;
      if(next){
        set_point = atoi(next);
      }

      if(select >= 0 && select < NUM_MOTORS && set_point){
        current_cmd.select = select;
        current_cmd.set_point = set_point;
        current_cmd.wait = 0;
        cmd_queue.enq(current_cmd);
      }
    }
  }

  if(!waiting_mode){
    //Dequeue the motor command
    //But if we're in waiting mode and the joints are still going, don't

    while(cmd_queue.size() > 0){
      current_cmd = cmd_queue.dq();
      if(current_cmd.wait){
        //wait and no more move_to's
        waiting_mode = 1;
        break;
      }else if(current_cmd.select >=0 && 
              select < NUM_MOTORS && 
              set_point > 0){
        joints[current_cmd.select]->move_to(current_cmd.set_point);
      }
    }
  }


  uint8_t all_ok = 1;
  for(i=0;i<NUM_MOTORS;i++){
    all_ok = joints[i]->update() && all_ok;
  }
  if(all_ok)
    waiting_mode = 0;  //All joints have finished moving


  delay(10);
}


