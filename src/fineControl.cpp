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
#include <math.h>

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
double KAngular=.008;
double thresholdAngle=0;
double thresholdDistance=0;



bool got_cdot=false;
geometry_msgs::Twist cdot;
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

dist=235-cenPose->y;
angle=350-cenPose->x; //CENTER
	
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "Downward_facing_camera_control");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 
ros::Subscriber cen_sub_;
ros::Publisher u_pub_;

cen_sub_ = nh_.subscribe<geometry_msgs::Point>("/ball_pixel",1, goalCB);
u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);

geometry_msgs::Twist finalVel;

//LAST VELOCITY INIT
lastVel.linear.x=10;
lastVel.angular.z=10;



while(ros::ok()){
	ros::spinOnce();

	if (gotInitialGoal==true){
		std::cout << "\n\n\n";
		std::cout << "Got measurement" <<  angle<<"\n";

		if (abs(dist)>thresholdDistance){
			finalVel.linear.x=KLinear*dist;
		}else{
			finalVel.linear.x=0;
		}

		if (abs(angle)>thresholdAngle){
			finalVel.angular.z=KAngular*angle;
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

		if (finalVel.linear.x>.1){
			finalVel.linear.x=.1;
		}else if (finalVel.linear.x<-.1){
			finalVel.linear.x=-.1;
		}


//		std::cout << "x1: " << x1 << "\n";
//		std::cout << "y2: " << y22 << "\n";
//		std::cout << "y1: " << y11 << "\n";
//		std::cout << "ceny: " << ceny << "\n";
//		std::cout << "cenx: " << cenx << "\n";
//		std::cout << "angle: " << angle << "\n";
//		std::cout << "angular"<< a <<"\n";
//		std::cout << "linear"<< finalVel.linear.x << "\n";
		u_pub_.publish(finalVel);
		loop_rate.sleep();

}

}
