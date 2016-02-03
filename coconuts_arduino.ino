/*

This controls the arm 
Input format over serial: <arm index> <set point>|
e.g. 
0 200|1 400|2 300|3 400

*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "JointController.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


// Select which 'port' M1, M2, M3 or M4. In this case, M1
JointController joint_0(80, 700, A0, AFMS.getMotor(1));
JointController joint_1(94, 797, A1, AFMS.getMotor(2));
JointController joint_2(340,750, A2, AFMS.getMotor(3));
JointController joint_3(240,640, A3, AFMS.getMotor(4), 10);

#define NUM_MOTORS 4
JointController* joints[] = {&joint_0, &joint_1, &joint_2, &joint_3};

#define BUF_SIZE 16
char buffer[BUF_SIZE];

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

  joint_3.move_to(240);
  joint_3.set_ki(0);
}

int select;
int set_point = 500;
int i;

void loop() {
  // Serial.println("hello from the other side");
  if (Serial.available()) {
    int read = readUntilPipe(buffer, BUF_SIZE);
    if(!read)
      return;

    if(buffer[0]=='a' || buffer[0]=='A'){
      //dump analog
      for(i=0; i<NUM_MOTORS; i++){
        Serial.print(i);
        Serial.print(" ");
        joints[i]->dump();
        Serial.print("|");
      }
    }else{
      select = atoi(buffer);
      char * next = strchr(buffer, ' ');
      set_point = 0;
      if(next){
        set_point = atoi(next);
      }

      if(select >= 0 && select < NUM_MOTORS && set_point){
        joints[select]->move_to(set_point);
      }
    }
  }

  for(i=0;i<NUM_MOTORS;i++)
    joints[i]->update();

  
  delay(100);
}



