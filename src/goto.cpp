/*

Go to a position function by Aaron Ma :DxD;))))

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <turtlebot_deployment/PoseWithName.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float32.h>
#include <math.h>

//Declare Variables
double y, x, r,x1,y11,x2,y22,x3,y33,cdotMag, cdotAngle;
double orientation;
double robVel_;
double OmegaC;
double OmegaD,dot,det,angle;
double cenx, ceny;
double dist=0;
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
bool got_cdot=false;
geometry_msgs::Twist cdot;
// Construct Node Class
using namespace std;



// Other member variables

geometry_msgs::Twist robVel;
geometry_msgs::Twist lastVel;
turtlebot_deployment::PoseWithName Pose;

bool got_vel_;
bool gotInitialGoal=false;


void goalCB(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
gotInitialGoal=true;
	if (cenPose->pose.position.z!=-1){
	cenx=cenPose->pose.position.x;
	ceny=cenPose->pose.position.y;
	}else{
	cenx=x;
	ceny=y;
	}
	
}

void tfCB(const tf2_msgs::TFMessage::ConstPtr& tf)
{

	if (tf->transforms[0].child_frame_id=="base_footprint" && tf->transforms[0].header.frame_id=="map"){
	orientation = tf::getYaw(tf->transforms[0].transform.rotation)+3.14/2;
	x1=sin(orientation);
	y11=cos(orientation);
	x=tf->transforms[0].transform.translation.x; 
	y=tf->transforms[0].transform.translation.y;

	x2=cenx-x;
	y22=ceny-y;
	}

}



int main(int argc, char **argv)
{

ros::init(argc, argv, "goto");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 
ros::Subscriber pos_sub_;
ros::Subscriber cen_sub_, cdot_sub_;
ros::Subscriber KTERM_sub;
ros::Subscriber CTERM_sub;
ros::Publisher u_pub_;

pos_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, tfCB);
cen_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/goal_pose",1, goalCB);

geometry_msgs::Twist finalVel;

//CDOT INIT
cdot.linear.x=0;
cdot.linear.y=0;
cdot.linear.z=0;


//LAST VELOCITY INIT
lastVel.linear.x=0;
lastVel.angular.z=0;

u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);

while(ros::ok()){
	ros::spinOnce();

	if (gotInitialGoal==true){

		dot = x1*x2 + y11*y22;
		det = x1*y22 - y11*x2;
		dist=sqrt(x2*x2+y22*y22);

		angle = atan2(det, dot);

			v=cos(angle)*V;
			if (dist<.1){
				v=0;
			}
	

//		if (cos(angle)>0){
//			if (abs(angle*180/3.14)<45){
//				a=A*angle;
//			}
//			else{
//				a=A;
//			}
//		}
//		else if (angle*180/3.14>90){
//			if (abs(angle*180/3.14)>135){
//				a=-A*(angle-3.14);
//			}
//			else{
//				a=A;
//			}
//		}
//		else if (angle*180/3.14<-90){
//			if (abs(angle*180/3.14)>135){
//				a=-A*(angle+3.14);
//			}
//			else{
//				a=A;
//			}
//		}
//	
a=A*angle;


//		if (dist<.01){
//			a=0;
//		}


			finalVel.linear.x=KTERM*v;
			finalVel.angular.z=a;

		}

		if (dist < .25){
				if ((finalVel.linear.x-lastVel.linear.x)>.01){
					finalVel.linear.x=lastVel.linear.x+.02;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.01){
					finalVel.linear.x=lastVel.linear.x-.02;
				}
		}
		else  {
				if ((finalVel.linear.x-lastVel.linear.x)>.05){
					finalVel.linear.x=lastVel.linear.x+.03;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.05){
					finalVel.linear.x=lastVel.linear.x-.03;
				}		
		}
				

		if ((finalVel.angular.z-lastVel.angular.z)>.001){
					finalVel.angular.z=lastVel.angular.z+.001;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.001){
					finalVel.angular.z=lastVel.angular.z-.001;
				}
		
		if (abs(finalVel.angular.z)<.0001){
		finalVel.angular.z=0;			
		}

		lastVel=finalVel;
		finalVel.linear.x=finalVel.linear.x/2;

		if (a>.5){
			a=.5;
		}else if (a<-.5){
			a=-.5;
		}

		if (finalVel.linear.x>.2){
			finalVel.linear.x=.2;
		}else if (finalVel.linear.x<0){
			finalVel.linear.x=0;
		}

		finalVel.angular.z=a;


		finalVel.angular.z=-finalVel.angular.z;

		std::cout << "\n\n\n";
		std::cout << "x2: " << x2 << "\n";
		std::cout << "x1: " << x1 << "\n";
		std::cout << "y2: " << y22 << "\n";
		std::cout << "y1: " << y11 << "\n";
		std::cout << "ceny: " << ceny << "\n";
		std::cout << "cenx: " << cenx << "\n";
		std::cout << "angle: " << angle << "\n";
		std::cout << "angular"<< a <<"\n";
		std::cout << "linear"<< finalVel.linear.x << "\n";
		u_pub_.publish(finalVel);
		loop_rate.sleep();

}

}
