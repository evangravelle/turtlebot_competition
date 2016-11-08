/*

Obstacles by Aaron Ma :DxD;))))

*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <coconuts_common/SensorStatus.h>
#include <coconuts_common/SensorReading.h>
#include <std_msgs/Float32.h>
#include <math.h>

//Declare Variables
tf2_msgs::TFMessage obstacles;
geometry_msgs::PoseStamped obstaclePose;

const int num=1;
const int numSensors=3;
const int OFF=0;
const int ON=1;
int DETECTED=1;
int NOT_DETECTED=0;

double thresholdMin=5;
double thresholdMax=100;
double orientation=0;


bool newMeasurement=true;

int r1=0;
int r2=0;
int r3=0;

double x=1;
double y=1;

// Construct Node Class
using namespace std;

void toPose(){
	x=0;
	y=0;
	std::cout << "Got Sensor Readings: " <<    obstacles.transforms[0].transform.translation.x <<" \n";
	obstaclePose.pose.position.x=x+obstacles.transforms[0].transform.translation.y*sin(orientation) + obstacles.transforms[0].transform.translation.x*cos(orientation);
	obstaclePose.pose.position.y=y+obstacles.transforms[0].transform.translation.y*cos(orientation) + obstacles.transforms[0].transform.translation.x*sin(orientation);
obstaclePose.pose.position.x=obstaclePose.pose.position.x*5;
obstaclePose.pose.position.y=obstaclePose.pose.position.y*5;
	obstaclePose.pose.orientation.w=1;
}

void tfCB(const tf2_msgs::TFMessage::ConstPtr& tf)
{

	if (tf->transforms[0].child_frame_id=="base_footprint" && tf->transforms[0].header.frame_id=="map"){
	orientation = tf::getYaw(tf->transforms[0].transform.rotation)+3.14/2;
	x=tf->transforms[0].transform.translation.x; 
	y=tf->transforms[0].transform.translation.y;
	}

}

void sonarCB(const coconuts_common::SensorStatus::ConstPtr& s){

r1=0;
r2=0;
r3=0;



//Later transform to global coordinates;
	if (s->sensor_readings[2].reading >thresholdMin && s->sensor_readings[2].reading < thresholdMax){
		r1=DETECTED;}

	if (s->sensor_readings[0].reading >thresholdMin && s->sensor_readings[0].reading < thresholdMax){
		r2=DETECTED;}

	if (s->sensor_readings[1].reading >thresholdMin && s->sensor_readings[1].reading < thresholdMax){
		r3=DETECTED;}

	if (r1==DETECTED){
		if (r2==DETECTED){
			if (r3==DETECTED){
					//LEFT AND CENTER AND RIGHT
					obstacles.transforms[0].transform.translation.y=0;
			}else{
					//LEFT AND CENTER
					obstacles.transforms[0].transform.translation.y=-.025;
			}
		}else{
			if (r3==DETECTED){
					//LEFT AND RIGHT
					obstacles.transforms[0].transform.translation.y=0;
			}else{
					//ONLY LEFT
					obstacles.transforms[0].transform.translation.y=-.05;
			}
		}
	}else{
		if (r2==DETECTED){
			if (r3==DETECTED){
					//CENTER AND RIGHT
					obstacles.transforms[0].transform.translation.y=.025;
			}
		}else{
			if (r3==DETECTED){
					//ONLY RIGHT
					obstacles.transforms[0].transform.translation.y=.05;
			}else{
					//NONE
			}
		}
	}




	if (r1==DETECTED || r2==DETECTED || r3==DETECTED){
		toPose();
		obstacles.transforms[0].transform.translation.x=s->sensor_readings[0].reading/1000.;
		newMeasurement=true;
	}else{
		toPose();
		newMeasurement=true;
	}


}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Obstacle_Filter");
	ros::NodeHandle ph_, nh_;
	ros::Rate loop_rate(5); 
	ros::Subscriber cen_sub_, tf_sub_;
	ros::Publisher u_pub_, pos_pub_;

	obstacles.transforms.resize(num);
	obstaclePose.header.frame_id="michelangelo";

	for (int i=1;i<num;i++){
		obstacles.transforms[i].transform.rotation.x=OFF;
	}

//Sensors~~
cen_sub_ = nh_.subscribe<coconuts_common::SensorStatus>("/sensor_status",1, sonarCB);
tf_sub_ = nh_.subscribe<tf2_msgs::TFMessage>("/tf", 1, tfCB);

//obstacle locations
u_pub_ = nh_.advertise<tf2_msgs::TFMessage>("/tf", 1, true);
pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("poseEstimation", 1, true);


while(ros::ok()){
	ros::spinOnce();
		if (newMeasurement==true){
			pos_pub_.publish(obstaclePose);
		}
		loop_rate.sleep();

}

}
