/*

Go to a position function by Aaron Ma :DxD;))))

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float32.h>
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/ControlState.h>
#include <math.h>
#include <states.h>

//Declare Variables
double y, x, r,x1,y11,x2,y22,x3,y33,cdotMag, cdotAngle;
double orientation;
double robVel_;
double OmegaC;
double cenx, ceny;
double v=0;
double V=.5;
double a=0;
double A=1;
double transformScale=900;
double cdotContribution=0;
double transformX=301;
double transformY=247;
float lastcdotAngle=0;
float CTERM=.2;
float KTERM=.5;
float CANGLETERM=.5;

double dist=0;
double angle=0;
double KLinear=.0005;
double ILinear=0;
double KILinear=0;//.00005;
double KAngular=.005;
double IAngular=0;
double KIAngular=0;//.0005;
double thresholdAngle=0;
double thresholdDistance=0;

double m=0;
double b=0;

int state=0;
bool goForBall=false;
bool got_cdot=false;
geometry_msgs::Twist cdot;
coconuts_common::ArmMovement grabBallOpen;
ros::Publisher control_pub;
coconuts_common::ControlState cs;

// Construct Node Class
using namespace std;



// Other member variables

geometry_msgs::Twist robVel;
geometry_msgs::Twist lastVel;

bool got_vel_;
bool gotInitialGoal=false;


void goalCB(const geometry_msgs::Point::ConstPtr& cenPose){

//Calculate Dist and angle
gotInitialGoal=true;

double xPrime=((480-cenPose->y)-b)/m;
dist=438-cenPose->y;
angle=xPrime-cenPose->x-15; //CENTER

	if (abs(dist)<10 && abs(angle) <5){
		state=0;
		cs.sub_state=CENTER_ON_BALL;
		control_pub.publish(cs);
	}

}

void stateCB(const coconuts_common::ControlState::ConstPtr& control_state){
	if (control_state -> state == MOVE_TO_BALL && control_state-> sub_state == MOVING_TO_BALL){ //&& control_state -> sub_state == MOVING_TO_BALL
		state=1;
	}
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "AaRoNmA");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 
ros::Subscriber cen_sub_, control_sub;
ros::Publisher u_pub_,m_pub;


control_sub = nh_.subscribe<coconuts_common::ControlState>("/control_state",1, stateCB);
cen_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1, goalCB);
u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
control_pub = nh_.advertise<coconuts_common::ControlState>("/control_substate",1, true);


geometry_msgs::Twist finalVel;

//LAST VELOCITY INIT
lastVel.linear.x=0;
lastVel.angular.z=0;

m=(((480-124)-(480-322.5))/(266-322.5));
b=(480-124)-m*(266);

while(ros::ok()){
	ros::spinOnce();
		std::cout << "state : " <<  state<<"\n";
	if (state==1){
	if (abs(angle)<5 && abs(dist)<5){
		goForBall=false;
	}

	if (gotInitialGoal==true && goForBall==false){

		ILinear=ILinear+dist;
		IAngular=IAngular+angle;

		if (ILinear > 25){
			ILinear=25;
		}else if (ILinear <-25){
			ILinear=-25;
		}

		if (IAngular > 25){
			IAngular=25;
		}else if (IAngular <-25){
			IAngular=-25;
		}

		if (abs(dist)>thresholdDistance){
			finalVel.linear.x=KLinear*dist+KILinear*ILinear;
		}else{
			finalVel.linear.x=0;
		}

		if (abs(angle)>thresholdAngle){
			finalVel.angular.z=KAngular*angle+KIAngular*IAngular;
		}else{
			finalVel.angular.z=0;
		}
		}





//				if ((finalVel.linear.x-lastVel.linear.x)>.01){
//					finalVel.linear.x=lastVel.linear.x+.01;
//				}	
//				else if ((finalVel.linear.x-lastVel.linear.x)<-.01){
//					finalVel.linear.x=lastVel.linear.x-.01;
//				}


//				if ((finalVel.angular.z-lastVel.angular.z)>.001){
//					finalVel.angular.z=lastVel.angular.z+.001;
//				}	
//				else if ((finalVel.linear.x-lastVel.linear.x)<-.001){
//					finalVel.angular.z=lastVel.angular.z-.001;
//				}
		
		if (abs(finalVel.angular.z)<.0001){
		finalVel.angular.z=0;			
		}

		lastVel=finalVel;


//Maxes
		if (finalVel.angular.z >.5){
			finalVel.angular.z=.5;
		}else if (finalVel.angular.z<-.5){
			finalVel.angular.z=-.5;
		}

		if (finalVel.linear.x>.2){
			finalVel.linear.x=.2;
		}else if (finalVel.linear.x<-.2){
			finalVel.linear.x=-.2;
		}
		std::cout <<"\n\n";
		std::cout << "Error x: " <<  angle<<"\n";
		std::cout << "Error y: " <<  dist<<"\n";

		if (goForBall==false||1==1){
			u_pub_.publish(finalVel);
		}else{
			m_pub.publish(grabBallOpen);
		}
	}



		loop_rate.sleep();

}

}




