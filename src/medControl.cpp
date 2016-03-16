/*

Go to a position function by Aaron Ma :DxD;)))),

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float32.h>
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/SensorStatus.h>
#include <math.h>
#include <states.h>

//~~~Declare Variables
double y, x, r,x1,y11,x2,y22,x3,y33;
double orientation;
double dot,det;

//~~~Define Define starting locations for the bucket and cocobot

int BUCKETLOCX;
int BUCKETLOCY;

//~~~blah blah blah
double cenx, ceny;
double v=0;
double V=.5;
double a=0;
double A=1;
double waypointX=0;
double waypointY=0;

double image_width=320.0;
double image_height=240.0;

float KTERM=.5;
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

int queueState=0;
bool left_obstacle=false;
bool right_obstacle=false;
bool center_obstacle=false;

coconuts_common::ArmMovement grabBallOpen;
ros::Publisher control_pub;
ros::Subscriber cen_sub_, cen2_sub_, control_sub,sensor_status_sub, cen3_sub_, cen4_sub_, cen5_sub_ , pos_sub_;
ros::Publisher u_pub_,m_pub;
coconuts_common::ControlState cs;

// Construct Node Class
using namespace std;



// Other member variables
geometry_msgs::Twist lastVel;
geometry_msgs::Twist finalVel;
geometry_msgs::PoseArray poseArray;

bool got_vel_;
bool gotInitialGoal=false;



//~~~Callback from detect_ball_forward/ball_pixel for approaching ballz
void goalCB(const geometry_msgs::Point::ConstPtr& cenPose){
	if (substate==1 && state==1){
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
			if (orange_or_green==0){
			cs.sub_state=CENTER_ON_ORANGE;
			}else{
			cs.sub_state=CENTER_ON_GREEN;
			}
			control_pub.publish(cs);
		}
	}
}

//~~~Callback for detect_ball_down/ball_pixel for centering on ballz
void goalCB2(const geometry_msgs::Point::ConstPtr& cenPose){
	if (substate==2 && cenPose->x >0 && state==1){
		gotInitialGoal=true;
		dist=235/2-cenPose->y;
		angle=350/2-cenPose->x; 
		if (abs(dist)<10 && abs(angle) <10){
			substate=3;
			if (orange_or_green==0){
			cs.sub_state=AT_ORANGE;
			}else{
			cs.sub_state=AT_GREEN;
			}
			control_pub.publish(cs);
		}
	}
}

//~~~Callback for the bucket! I think
void goalCB3(const geometry_msgs::Point::ConstPtr& cenPose){
                        // cout << "got bucket pixel" << "\n";
	if (state==4 && (substate==1 || substate==2)){

		if (cenPose->x > 0){
			substate=2;
		}

	double xPrime=((230-cenPose->y)-b)/m;
		dist=230-cenPose->y;
		angle=xPrime-cenPose->x-25; 

		if  (abs(dist)<45){
			substate=8;
		}

		if (abs(dist)<15 && abs(angle) <15){
		state=10;
		substate=1;
		cs.sub_state=AT_GOAL;
		control_pub.publish(cs);
		}
	}
}

//~~~Queue
void goalCB5(const geometry_msgs::Pose::ConstPtr& cenPose){
	if (state==4 || state==2){
		queueState=1;
		waypointX=cenPose->position.x;
		waypointY=cenPose->position.y;
	}
}

//~~~Callback for the global location NCU
void goalCB4(const geometry_msgs::PoseStamped::ConstPtr& cenPose){
//OBSOLETE
//        gotInitialGoal=true;
//	cenx=bucketLocX/112.5;
//	ceny=-bucketLocY/112.5;
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

//~~~Callback for the cocobots position in meter coordinates
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
		//OBSOLETE FOR THE MOMENT
		if (state==4 && substate==3 && 1==2){
			if (abs(x-poseArray.poses[queueState].position.x)<.1 && abs(y-poseArray.poses[queueState].position.y)<.1 ){
				queueState=queueState+1;
				if (poseArray.poses[queueState].position.x==0){
					substate=1;
				}
			}
		}
	}

}

//~~~Callback to find out what the heck we are doing
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
		if (state==1 || state==0){
			substate=1;
		}
		state=4;
	}else if (control_state -> sub_state ==MOVING_TO_GOAL){
		if (state==1 || state==0){
			substate=1;
		}
		if (control_state-> sub_state == SEARCH_FOR_GOAL){
			substate=1;
		}
		state=4;
		gotInitialGoal=true;
	}

	else if(control_state -> state == NEXT_RUN_PREP){
		state=10;
	}
	else {
		state=0;
	}
}

//~~~Callback for the ultrasonics, gotta go fast
void sensorCB(const coconuts_common::SensorStatus::ConstPtr& sensor_msg) {

	if (substate==8){

		int center_sonar = 0;
		int right_sonar = 0;
		int left_sonar = 0;

		// int RL_goal = 19;
		double ANGLE_BOOST = 16;
		double DIST_BOOST = 16;

		bool sonar_ok = true;
		for(int i=0;i<sensor_msg->sensor_readings.size();i++){
			int reading = sensor_msg->sensor_readings[i].reading;
			int sensor = sensor_msg->sensor_readings[i].sensor;

			if(sensor==0)
				center_sonar = reading;
			else if(sensor==1)
				right_sonar = reading;
			else if(sensor==2)
				left_sonar = reading;
		}

		if(center_sonar > 0 && right_sonar > 0 && left_sonar > 0){
			dist = center_sonar - 6;
			dist *= DIST_BOOST;

			//Both sensors should be about 19
			angle = ((right_sonar - 19) + (19 - left_sonar))/2.0;
			angle *= ANGLE_BOOST;
			if (center_sonar >= 6 && center_sonar <= 7
				&& right_sonar >= 16 && right_sonar <= 20
				&& left_sonar >= 16 && right_sonar <= 20){
                state=10;
		        substate=1;
		        cs.sub_state=AT_GOAL;
		        control_pub.publish(cs);

			}
		}else if(right_sonar > 0 && center_sonar > 0){
			angle = right_sonar - 19;
			angle *= ANGLE_BOOST;
		}else if(left_sonar > 0 && center_sonar > 0){
			angle = 19 - left_sonar;
			angle *= ANGLE_BOOST;
		}else{
			ROS_WARN("Sonar is fucked! Where's the bucket??");
		}
		ROS_INFO("Sonar centering: angle = [%f],  dist=[%f]",angle,dist);
	}

	// TODO define these magic numbers and parameterize the reading value
	for (int i = 0; i < sensor_msg->sensor_readings.size(); i++ ) {
	    //ROS_INFO("Explorer: Looking at sensor [%d] reading [%d].", sensor_msg->sensor_readings[i].sensor, sensor_msg->sensor_readings[i].reading);

		if(sensor_msg->sensor_readings[i].reading == 0)
			continue; 	//Sensor reading is invalid, skip it

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

//~~~medium distance control, for ballz and bucket approaching
void medControl(){
KIAngular=0;
KILinear=0;
KLinear=.00065;
KAngular=.0065;

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

//~~~Fine adjustment control for centering on ballz
void fineControl(){
KIAngular=0.005;
KILinear=0.00145;
KLinear=.0008;
KAngular=.08;

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


//~~~ explore function ncu
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

void setGlobalGoal(double targetX, double targetY){
	gotInitialGoal=true;
	cenx=targetX;
	ceny=targetY;
}

//~~~global control function, goes to a position cenx and ceny
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

		if (left_obstacle==true){
			finalVel.linear.x=0;
			finalVel.angular.z=-.3;
		}else if (right_obstacle==true){
			finalVel.linear.x=0;
			finalVel.angular.z=.3;
		}

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
				

		if ((finalVel.angular.z-lastVel.angular.z)>.003){
					finalVel.angular.z=lastVel.angular.z+.003;
				}	
				else if ((finalVel.linear.x-lastVel.linear.x)<-.003){
					finalVel.angular.z=lastVel.angular.z-.003;
				}
		
		if (abs(finalVel.angular.z)<.0001){
		finalVel.angular.z=0;			
		}

		lastVel=finalVel;
		finalVel.linear.x=finalVel.linear.x/2;

		if (a>.3){
			a=.3;
		}else if (a<-.3){
			a=-.3;
		}

		if (finalVel.linear.x>.15){
			finalVel.linear.x=.15;
		}else if (finalVel.linear.x<0){
			finalVel.linear.x=0;
		}

		finalVel.angular.z=a;


		finalVel.angular.z=-finalVel.angular.z;

                        if (dist<.1){
                                finalVel.linear.x=0;
				finalVel.angular.z=.25;
                        }


}


//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~//~~~
int main(int argc, char **argv)
{
ros::init(argc, argv, "AaRoNmA");
ros::NodeHandle ph_, nh_;
ros::Rate loop_rate(50); 


control_sub = nh_.subscribe<coconuts_common::ControlState>("/control_state",1, stateCB);
cen_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1, goalCB);
cen2_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_ball_down/ball_pixel",1, goalCB2);
cen5_sub_ = nh_.subscribe<geometry_msgs::Pose>("/waypoint",1, goalCB5);
cen3_sub_ = nh_.subscribe<geometry_msgs::Point>("/detect_bucket_forward/bucket_pixel",1, goalCB3);
u_pub_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, true);
control_pub = nh_.advertise<coconuts_common::ControlState>("/control_substate",1, true);
sensor_status_sub  = nh_.subscribe<coconuts_common::SensorStatus>("sensor_status",1,sensorCB);
cen4_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/goal_pose",1, goalCB4);
pos_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, tfCB);



//LAST VELOCITY INIT
lastVel.linear.x=0;
lastVel.angular.z=0;

m=(((480-124)-(480-322.5))/(266-322.5));
b=(480-124)-m*(266);


    if (nh_.getParam("/medControl/bucketLocX", BUCKETLOCX))
    {
      ROS_INFO("Got BUCKET LOCATION");
    }
    else
    {
      ROS_ERROR("Failed to get param BUCKET LOCATION");
    }

    nh_.getParam("/medControl/bucketLocY", BUCKETLOCY);

        ceny=-BUCKETLOCY/112.5;
        cenx=BUCKETLOCX/112.5;



while(ros::ok()){
	finalVel.linear.x=0;
	finalVel.angular.z=0;

	ros::spinOnce();
		// std::cout << "state : " <<  state<<"\n";
		// std::cout << "substate : " << substate << "\n"; 	
	
	if (queueState==1){
		if (dist < .2){
			queueState=0;
		}
		setGlobalGoal(waypointX,waypointY);
		global();
	} else{
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
		setGlobalGoal(400/112.5,-400/112.5);
		global();
	}else if (state==3){
		cout << "Global ~~~" << "\n";
	}else if (state==4){
		// cout << "BUCKET \n";
		if (substate==1){
			setGlobalGoal(BUCKETLOCX/112.5,-BUCKETLOCY/112.5);
			global();
		}else if (substate==2 || substate==8){
			medControl();
		}else if (substate==3){
//			setGlobalGoal(poseArray.poses[queueState].position.x,poseArray.poses[queueState].position.y);
			global();
		}

	}else if (state==5){
//		cout << "GLOBAL \n";
		global();
	}
	else{
//		cout << "inactive" << "\n";
	}
	if (state!=10){
        u_pub_.publish(finalVel);
	}
	}
	loop_rate.sleep();

}

}
