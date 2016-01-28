
#include <stdio.h>
#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 




// Keep track of Quadcopter state
double T = 30; // ROS loop rate


stereo_msgs::DisparityImage AaronIsCool;
geometry_msgs::Pose ball;

const int height=480;
const int width =640;
const int AVG_SIZE=50;
float avg[AVG_SIZE];
float finalAverage=0;

float returnDistance(float measurement){


	if (measurement<1 && measurement >-1){
	finalAverage=0;
		for (int i=AVG_SIZE-1;i>0;i--){
			avg[i]=avg[i-1];
			finalAverage=finalAverage+avg[i];
		}
	avg[0]=measurement;
	finalAverage=finalAverage+avg[0];
	finalAverage=finalAverage/AVG_SIZE;
	}

	return (finalAverage);

}

void distanceCB(const stereo_msgs::DisparityImage::ConstPtr& imagePtr)
{
	AaronIsCool.image.data=imagePtr->image.data;
	AaronIsCool.f=imagePtr->f;
	AaronIsCool.T=imagePtr->T;
//	std::cout<<"Disparity: " << (int)AaronIsCool.image.data[(int) (240*640+320)*4+1]  << "\n";
}

void locateBall(const geometry_msgs::Pose::ConstPtr& posePtr)
{


	ball.orientation.z=45-90*(posePtr->position.x/width);
	ball.position.z=returnDistance((-AaronIsCool.f*AaronIsCool.T)/(int)AaronIsCool.image.data[(int) (posePtr->position.y*width+posePtr->position.x)*4+2]);

	std::cout<<"Ball Location : " << (int)AaronIsCool.image.data[(int) (posePtr->position.y*width+posePtr->position.x)*4+2] << "\n";

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereoDisparitytoDistance"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    ros::NodeHandle n;


	ros::Subscriber cameraImageLeft=n.subscribe<stereo_msgs::DisparityImage>("/stereo/disparity", 1, distanceCB);
	ros::Subscriber receivedPoint=n.subscribe<geometry_msgs::Pose>("/ballLocation",1, locateBall);
	ros::Publisher ballMeasurementPublisher=n.advertise<geometry_msgs::Pose>("/ballMeasurement",1, true);

ros::Time stamp = ros::Time::now();
AaronIsCool.image.data.resize(4*height*width);

    while (ros::ok()) 
    {
	ros::spinOnce();
	std::cout<<"Distance: " <<ball.position.z<< "\n";
	ballMeasurementPublisher.publish(ball);
        loop_rate.sleep();
    }
}
//END

