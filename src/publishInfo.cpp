
#include <stdio.h>
#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <fstream>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>





// Keep track of Quadcopter state
double T = 5; // ROS loop rate

sensor_msgs::CameraInfo camLeft;
sensor_msgs::CameraInfo camRight;
sensor_msgs::Image leftImage;
sensor_msgs::Image rightImage;



bool gotLeft=false;
bool gotRight=false;

void leftImageCB(const sensor_msgs::Image::ConstPtr& imagePtr)
{
	leftImage.header=imagePtr->header;
	leftImage.height=imagePtr->height;
	leftImage.width=imagePtr->width;
	leftImage.encoding=imagePtr->encoding;
	leftImage.is_bigendian=imagePtr->is_bigendian;
	leftImage.step=imagePtr->step;
	leftImage.data=imagePtr->data;
	gotLeft=true;

}

void rightImageCB(const sensor_msgs::Image::ConstPtr& imagePtr)
{
	rightImage.header=imagePtr->header;
	rightImage.height=imagePtr->height;
	rightImage.width=imagePtr->width;
	rightImage.encoding=imagePtr->encoding;
	rightImage.is_bigendian=imagePtr->is_bigendian;
	rightImage.step=imagePtr->step;
	rightImage.data=imagePtr->data;
	gotRight=true;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "beepboop"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    ros::NodeHandle n;

camLeft.D.resize(5);
camRight.D.resize(5);

camLeft.height=480;
camLeft.width=640;
camLeft.distortion_model="plumb_bob";


camLeft.D.resize(5);
camRight.D.resize(5);

camLeft.height=480;
camLeft.width=640;
camLeft.distortion_model="plumb_bob";

camLeft.D[0]=0.131375;
camLeft.D[1]=-0.242915;
camLeft.D[2]=0.000130;
camLeft.D[3]=0.002108;
camLeft.D[4]=0.000000;




camLeft.K[0]=623.975703;
camLeft.K[1]=0;
camLeft.K[2]=323.778395;

camLeft.K[3]=0;
camLeft.K[4]=624.295423;
camLeft.K[5]=242.136119;

camLeft.K[6]=0;
camLeft.K[7]=0;
camLeft.K[8]=1;



camLeft.R[0]=1;
camLeft.R[1]= 0;
camLeft.R[2]= 0;

camLeft.R[3]=0;
camLeft.R[4]= 1;
camLeft.R[5]= 0;

camLeft.R[6]= 0;
camLeft.R[7]= 0;
camLeft.R[8]=1;


camLeft.P[0]=634.055725;
camLeft.P[1]=0.000000;
camLeft.P[2]=324.300872;
camLeft.P[3]=0.000000;

camLeft.P[4]=0;
camLeft.P[5]=635.019104;
camLeft.P[6]=241.680174;
camLeft.P[7]=0;

camLeft.P[8]=0;
camLeft.P[9]=0;
camLeft.P[10]=1;
camLeft.P[11]=0;


camRight.height=480;
camRight.width=640;
camRight.distortion_model="plumb_bob";


camRight=camLeft;


    ros::Subscriber cameraImageLeft=n.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, leftImageCB);
    ros::Subscriber cameraImageRight=n.subscribe<sensor_msgs::Image>("/camera_2/image_raw", 1, rightImageCB);
    ros::Publisher cameraInfoLeft = n.advertise<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 1000, true);
    ros::Publisher cameraInfoRight = n.advertise<sensor_msgs::CameraInfo>("/camera_2/camera_info", 1000, true);
    ros::Publisher cameraReLeft = n.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1000, true);
    ros::Publisher cameraReRight = n.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1000, true);



ros::Time stamp = ros::Time::now();


    while (ros::ok()) 
    {
	//ros::spinOnce();
	if (gotLeft==true && gotRight==true){
	stamp = ros::Time::now();

	leftImage.header.stamp=stamp;
	rightImage.header.stamp=stamp;
	camLeft.header.stamp=stamp;
	camRight.header.stamp=stamp;

	//cameraReLeft.publish(leftImage);
	//cameraReRight.publish(rightImage);
	cameraInfoLeft.publish(camLeft);
	cameraInfoRight.publish(camRight);

	gotLeft=false;
	gotRight=false;
	}
        cameraInfoLeft.publish(camLeft);
        cameraInfoRight.publish(camRight);


        loop_rate.sleep();
    }
}
//END

