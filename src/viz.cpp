

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>

using namespace std;

tf2_msgs::TFMessage poseEstimation; // Where the Quadcopter thinks it is
geometry_msgs::Twist twist;
geometry_msgs::Pose dummyPose;
visualization_msgs::Marker m;
float yaw=0;
//geometry_msgs::Twist command;
ros::Publisher locationPub;

double T = 50; // ROS loop rate

void controlCallback(const geometry_msgs::Twist::ConstPtr& command)
{

twist.linear=command->linear;
twist.angular=command->angular;

}

void poseCB(const geometry_msgs::Pose::ConstPtr& posePtr)
{

m.pose.position.x=cos((posePtr->orientation.z)*0.0174533)*(posePtr->position.z*10+1);
m.pose.position.y=sin((posePtr->orientation.z)*0.0174533)*(posePtr->position.z*10+1);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    ros::NodeHandle n;

    ros::Subscriber controlSub;

poseEstimation.transforms.resize(1);
poseEstimation.transforms[0].header.frame_id="odom";
poseEstimation.transforms[0].child_frame_id="base_footprint";


    controlSub = n.subscribe<geometry_msgs::Twist>("/move_ball", 1, controlCallback);
    locationPub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1000, true);
	ros::Subscriber ballLocationSub = n.subscribe<geometry_msgs::Pose>("/ballMeasurement", 1, poseCB);



m.header.frame_id = "base_footprint";

m.header.stamp = ros::Time();
m.id = 0;
m.type = visualization_msgs::Marker::SPHERE;
m.action = visualization_msgs::Marker::ADD;
m.pose.position.x = 1.1;
m.pose.position.y = 1.1;
m.pose.position.z = 0;
m.pose.orientation.x = 0.0;
m.pose.orientation.y = 0.0;
m.pose.orientation.z = 0.0;
m.pose.orientation.w = 1.0;
m.scale.x = .1;
m.scale.y = .1;
m.scale.z = .1;
m.color.a = 1.0; // Don't forget to set the alpha!
m.color.r = 0.0;
m.color.g = 1.0;
m.color.b = 0.0;


    while (ros::ok()) 
    {
        
        ros::spinOnce();

//
	poseEstimation.transforms[0].transform.translation.x=poseEstimation.transforms[0].transform.translation.x+twist.linear.x/T*cos(yaw);
	poseEstimation.transforms[0].transform.translation.y=poseEstimation.transforms[0].transform.translation.y+twist.linear.x/T*sin(yaw);

	yaw=yaw+twist.angular.z/T;
	dummyPose.orientation=tf::createQuaternionMsgFromYaw(yaw+3.14);


	poseEstimation.transforms[0].transform.rotation.z=dummyPose.orientation.z;
	poseEstimation.transforms[0].transform.rotation.x=dummyPose.orientation.x;
	poseEstimation.transforms[0].transform.rotation.y=dummyPose.orientation.y;
	poseEstimation.transforms[0].transform.rotation.w=dummyPose.orientation.w;

//m.pose.position.x=poseEstimation.transforms[0].transform.translation.x;
//m.pose.position.y=poseEstimation.transforms[0].transform.translation.y;

//
locationPub.publish(m);
        loop_rate.sleep();
    }
}
//END
