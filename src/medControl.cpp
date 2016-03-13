/*

Go to a position function by Aaron Ma :DxD;))))

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float32.h>
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/SensorStatus.h>
#include <math.h>
#include <states.h>

//Declare Variables
double y, x, r,x1,y11,x2,y22,x3,y33,cdotMag, cdotAngle;
double orientation;
double robVel_;
double OmegaC,dot,det;

double startingLocX=130;
double startingLocY=130;

double bucketLocX=130;
double bucketLocY=130;

double cenx, ceny;
double v=0;
double V=.5;
double a=0;
double A=1;
double transformScale=900;
double cdotContribution=0;
double transformX=301;
double image_width=320.0;
double image_height=240.0;
double transformY=247;
float lastcdotAngle=0;
float CTERM=.2;
float KTERM=.5;
float CANGLETERM=.5;


int orange_or_green=0;
double dist=0;
double angle=0;
double KLinear=.00065;
double ILinear=0;
double KILinear=0;//.00005;
double KAngular=.0065;
double IAngular=0;
double KIAngular=.0002;//.0005;
double thresholdAngle=0;
double thresholdDistance=0;

double m=0;
double b=0;

int state=0;
int substate=1;
bool goForBall=false;
bool got_cdot=false;

    bool left_obstacle=false;
    bool right_obstacle=false;
    bool center_obstacle=false;

geometry_msgs::Twist cdot;
coconuts_common::ArmMovement grabBallOpen;
ros::Publisher control_pub;
ros::Subscriber cen_sub_, cen2_sub_, control_sub,sensor_status_sub, cen3_sub_, cen4_sub_, pos_sub_;
ros::Publisher u_pub_,m_pub;
coconuts_common::ControlState cs;

// Construct Node Class
using namespace std;



// Other member variables

geometry_msgs::Twist robVel;
geometry_msgs::Twist lastVel;
geometry_msgs::Twist finalVel;

bool got_vel_;
bool gotInitialGoal=false;


void goalCB(const geometry_msgs::Point::ConstPtr& cenPose){
	if (substate==1){
	gotInitialGoal=true;
	double xPrime=((480-cenPose->y)-b)/m;
		dist=438-cenPose->y;
		angle=xPrime-cenPose->x; //CENTER

		if (abs(dist)<10 && abs(angle) <5){
			substate=2;
			ILinear=0;
			IAngular=0;
			angle=200;
			dist=0;
			cs.sub_state=CENTER_ON_ORANGE;
			control_pub.publish(cs);
		}
	}
}

void goalCB2(const geometry_msgs::Point::ConstPtr& cenPose){
	if (substate==2 && cenPose->x >0){
		gotInitialGoal=true;
		dist=235/2-cenPose->y;
		angle=350/2-cenPose->x; 
		if (abs(dist)<10 && abs(angle) <10){
			substate=3;
			cs.sub_state=AT_ORANGE;
			control_pub.publish(cs);
		}
	}
}

void goalCB3(const geometry_msgs::Point::ConstPtr& cenPose){
        if (state==5 &&  cenPose->x >0){ 
       double xPrime=((240-cenPose->y)-b)/m;
                dist=240-cenPose->y;
                angle=xPrime-cenPose->x; //CENTER

                if (abs(dist)<10 && abs(angle) <5){
                        substate=3;
                        cs.sub_state=AT_GOAL;
            //            control_pub.publish(cs);
                }
        }
}

void goalCB4(const geometry_msgs::PoseStamped::ConstPtr& cenPose){

        gotInitialGoal=true;
	cenx=bucketLocX/112.5;
	ceny=-bucketLocY/112.5;
//	if (state==4){
//		if (cenPose->pose.position.z!=-1){
//			cenx=cenPose->pose.position.x;
//			ceny=cenPose->pose.position.y;
//		}else{
//			cenx=x;
//			ceny=y;
//		}
	//}
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


void stateCB(const coconuts_common::ControlState::ConstPtr& control_state){
if (control_state -> state == MOVE_TO_BALL || control_state -> sub_state == MOVING_TO_ORANGE || control_state -> sub_state == MOVING_TO_GREEN){ //&& control_state -> sub_state == MOVING_TO_BALL
		if (control_state -> sub_state ==MOVING_TO_ORANGE){
			orange_or_green=0;
		}else if(control_state -> sub_state ==MOVING_TO_GREEN){
			orange_or_green=1;
		}
		if (state==2){
			substate=1;
		}
		state=1;
	}else if (control_state -> state ==FIND_BALL){
		state=2;
	}else if (control_state -> state ==FIND_GOAL){
                state=4;
        }else if (control_state -> sub_state ==MOVING_TO_GOAL){
		state=5;
		gotInitialGoal=true;
	}

	else{
		state=0;
	}
}

void sensorCB(const coconuts_common::SensorStatus::ConstPtr& sensor_msg) {

        // TODO define these magic numbers and parameterize the reading value
        for (int i = 0; i < sensor_msg->sensor_readings.size(); i++ ) {
            //ROS_INFO("Explorer: Looking at sensor [%d] reading [%d].", sensor_msg->sensor_readings[i].sensor, sensor_msg->sensor_readings[i].reading);

            switch (sensor_msg->sensor_readings[i].sensor) {
                case 0:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 5) {
                        center_obstacle = true;
                    } else {
                        center_obstacle = false;
                    }
                    break;

                case 1:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 40) {
                        right_obstacle = true;
                    } else {
                        right_obstacle = false;
                    }
                    break;

                case 2:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 40) {
                        left_obstacle = true;
                    } else {
                        left_obstacle = false;
                    }
                    break;
            
                default:
                    break;
            }
        }
}


void medControl(){
KIAngular=0;
KILinear=0;
	if (abs(angle)<5 && abs(dist)<5){
		goForBall=false;
	}

	if (gotInitialGoal==true && goForBall==false){


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

		if (finalVel.linear.x>.15){
			finalVel.linear.x=.15;
		}else if (finalVel.linear.x<-.15){
			finalVel.linear.x=-.15;
		}
		std::cout <<"\n\n";
		std::cout << "Error x: " <<  angle<<"\n";
		std::cout << "Error y: " <<  dist<<"\n";

}

void fineControl(){
KIAngular=0.001;
KILinear=0.001;
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


		if (abs(finalVel.angular.z)<.0001){
			finalVel.angular.z=0;
		}

		lastVel=finalVel;


//Maxes
		if (finalVel.angular.z >.15){
			finalVel.angular.z=.15;
		}else if (finalVel.angular.z<-.15){
			finalVel.angular.z=-.15;
		}

		if (finalVel.linear.x>.05){
			finalVel.linear.x=.05;
		}else if (finalVel.linear.x<-.05){
			finalVel.linear.x=-.05;
		}
		std::cout <<"\n\n";
		std::cout << "Error x: " <<  angle<<"\n";
		std::cout << "Error y: " <<  dist<<"\n";

        // ON SUCCESS THIS SHOULD PUBLISH SUBSTATE "AT_BALL"

}



void explore(){
//        finalVel.linear.x = 0.1;
//       finalVel.angular.z = 0.0;
//       if (left_obstacle && right_obstacle) {
//           finalVel.linear.x = -0.3; // Go Backwards
//        } else {
//           if (left_obstacle) {
//                finalVel.angular.z = -1.0; // rotate right
//                finalVel.linear.x = 0.0;
//            }
//            else if (right_obstacle) {
//                finalVel.angular.z = 1.0; // rotate left
//                finalVel.linear.x = 0.0;
//            }
//        }
}

void global(){
dot = x1*x2 + y11*y22;
		det = x1*y22 - y11*x2;
		dist=sqrt(x2*x2+y22*y22);

		angle = atan2(det, dot);

			v=cos(angle)*V;
			if (dist<.1){
				v=0;
			}
	

a=A*angle;



			finalVel.linear.x=KTERM*v;
			finalVel.angular.z=a;

		

		if (dist < .25){
				if ((finalVel.linear.x-lastVel.linear.x)>.02){
					finalVel.linear.x=lastVel.linear.x+.02;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.02){
					finalVel.linear.x=lastVel.linear.x-.02;
				}
		}
		else  {
				if ((finalVel.linear.x-lastVel.linear.x)>.03){
					finalVel.linear.x=lastVel.linear.x+.03;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.03){
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

}

int main(int argc, char **argv)
{
ros::init(argc, argv, "AaRoNmA");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 


control_sub = nh_.subscribe<coconuts_common::ControlState>("/control_state",1, stateCB);
cen_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1, goalCB);
cen2_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_ball_down/ball_pixel",1, goalCB2);
cen3_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_bucket_foward/bucket_pixel",1, goalCB3);
u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
control_pub = nh_.advertise<coconuts_common::ControlState>("/control_substate",1, true);
sensor_status_sub  = nh_.subscribe<coconuts_common::SensorStatus>("sensor_status",1,sensorCB);
cen4_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/goal_pose",1, goalCB4);
pos_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, tfCB);



        cenx=bucketLocX/112.5;
        ceny=-bucketLocY/112.5;
//LAST VELOCITY INIT
lastVel.linear.x=0;
lastVel.angular.z=0;

m=(((480-124)-(480-322.5))/(266-322.5));
b=(480-124)-m*(266);

while(ros::ok()){
	finalVel.linear.x=0;
	finalVel.angular.z=0;

	ros::spinOnce();
		std::cout << "state : " <<  state<<"\n";
	
	if (state==1){
		if (substate==1){
			cout << "subsate: 1" << "\n";
			medControl();
		}else if (substate==2){
			cout << "substate: 2" << "\n";
			fineControl();
		}
		u_pub_.publish(finalVel);
	}else if (state==2){
		cout << "Exploring ~~~" << "\n";
		explore();
	}else if (state==3){
		cout << "Global ~~~" << "\n";
	}else if (state==4){
		cout << "BUCKET \n";
		global();
	}else if (state==5){
		cout << "GLOBAL \n";
		global();
	}
	else{
		cout << "inactive" << "\n";
	}

	u_pub_.publish(finalVel);
	loop_rate.sleep();

}

}
