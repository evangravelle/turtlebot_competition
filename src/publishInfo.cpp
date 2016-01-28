
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
double T = 30; // ROS loop rate

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



	
camLeft.D[0]=0.161257;
camLeft.D[1]=-0.236007;
camLeft.D[2]=0.003009;
camLeft.D[3]= -0.007596;
camLeft.D[4]=0.000000;
	
camLeft.K[0]=626.355186;
camLeft.K[1]=0;
camLeft.K[2]=310.325584;

camLeft.K[3]=0;
camLeft.K[4]=625.619903;
camLeft.K[5]=247.641920;

camLeft.K[6]=0;
camLeft.K[7]=0;
camLeft.K[8]=1;

camLeft.R[0]=0.997930;
camLeft.R[1]= 0.004897;
camLeft.R[2]=-0.064125;

camLeft.R[3]=-0.004149;
camLeft.R[4]=0.999922;
camLeft.R[5]=0.011781;

camLeft.R[6]=0.064178;
camLeft.R[7]=-0.011491;
camLeft.R[8]=0.997872;

camLeft.P[0]=676.945399;
camLeft.P[1]=0.000000;
camLeft.P[2]=299.503029;
camLeft.P[3]=0.000000;

camLeft.P[4]=0;
camLeft.P[5]=676.945399;
camLeft.P[6]=248.725296;
camLeft.P[7]=0;

camLeft.P[8]=0;
camLeft.P[9]=0;
camLeft.P[10]=1;
camLeft.P[11]=0;


camRight.height=480;
camRight.width=640;
camRight.distortion_model="plumb_bob";


camRight.D[0]=0.150790;
camRight.D[1]= -0.252654;
camRight.D[2]=0.005241;
camRight.D[3]=-0.000414;
camRight.D[4]=0.000000;
	
camRight.K[0]=623.409081;
camRight.K[1]=0;
camRight.K[2]=314.934640;

camRight.K[3]=0;
camRight.K[4]=623.366784;
camRight.K[5]=246.519305;

camRight.K[6]=0;
camRight.K[7]=0;
camRight.K[8]=1;


camRight.R[0]=0.998957;
camRight.R[1]= -0.006288;
camRight.R[2]= -0.045218;

camRight.R[3]=0.005761;
camRight.R[4]=0.999914;
camRight.R[5]= -0.011775;

camRight.R[6]=0.045288;
camRight.R[7]= 0.011502;
camRight.R[8]=0.998908;

camRight.P[0]=676.945399;
camRight.P[1]=0.000000;
camRight.P[2]=299.503029;
camRight.P[3]=-71.123538;

camRight.P[4]=0;
camRight.P[5]=676.945399;
camRight.P[6]=248.725296;
camRight.P[7]=0;

camRight.P[8]=0;
camRight.P[9]=0;
camRight.P[10]=1;
camRight.P[11]=0;
    ros::Subscriber cameraImageLeft=n.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, leftImageCB);
    ros::Subscriber cameraImageRight=n.subscribe<sensor_msgs::Image>("/camera_2/image_raw", 1, rightImageCB);
    ros::Publisher cameraInfoLeft = n.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1000, true);
    ros::Publisher cameraInfoRight = n.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1000, true);
    ros::Publisher cameraReLeft = n.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1000, true);
    ros::Publisher cameraReRight = n.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1000, true);



ros::Time stamp = ros::Time::now();


    while (ros::ok()) 
    {
	ros::spinOnce();
	if (gotLeft==true && gotRight==true){
	stamp = ros::Time::now();

	leftImage.header.stamp=stamp;
	rightImage.header.stamp=stamp;
	camLeft.header.stamp=stamp;
	camRight.header.stamp=stamp;

	cameraReLeft.publish(leftImage);
	cameraReRight.publish(rightImage);
	cameraInfoLeft.publish(camLeft);
	cameraInfoRight.publish(camRight);

	gotLeft=false;
	gotRight=false;
	}


        loop_rate.sleep();
    }
}
//END

