

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
// #include <turtlebot_deployment/PoseWithName.h>
#include <tf/tf.h>
#include <math.h>
#include <termios.h>
#include <time.h> 

//Declare Variables
double x2, x1, r;
double orientation;
double robVel_;
double OmegaC;
double OmegaD;
double cenx, ceny;
// Construct Node Class

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

geometry_msgs::Twist twist;

int main(int argc, char **argv)
{
ros::init(argc, argv, "centroidKeyboard");
cenx=300;
ceny=200;
r=75;

ros::NodeHandle ph_("~"), nh_;
ros::Publisher cen_pub_;
geometry_msgs::Twist cmd_vel_;
// turtlebot_deployment::PoseWithName cenPose;
// cenPose.pose.position.x=cenx;
// cenPose.pose.position.y=ceny;
cen_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 5, true);

double k=1.75;
double u1=robVel_;
double u2=robVel_/r;
OmegaC=2;
OmegaD=1;
std::cout<<"Exiting Main Sequence: \n";
while(1==1){

twist.linear.x=0;
twist.angular.z=0;

	int c = getch();   // call your non-blocking input function
  if (c == 'd'){
    twist.angular.z=-.5;}
  else if (c == 'w'){
    twist.linear.x=.1;}
    else if (c == 's'){
    twist.linear.x=-.1;}
    else if (c == 'a'){
    twist.angular.z=.5;}
  else if (c == 'q'){
    twist.linear.x=.1;
    twist.angular.z=.5;}
  else if (c == 'e'){
    twist.linear.x=.1;
    twist.angular.z=-.5;}

cen_pub_.publish(twist);
    usleep(100);
}	
}

    

