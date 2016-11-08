//Template matching localization 
//Coconuts

#include <stdio.h>
#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <sstream>

int T=1;
static const char WINDOW1[] = "map";
static const char WINDOW2[] = "temp";
using namespace cv;
Mat result;








Point tMatch(Mat map, Mat temp, int result_rows, int result_cols ){

	double minVal; double maxVal; Point minLoc; Point maxLoc;

	result.create( result_rows, result_cols, CV_32FC1 );
	matchTemplate( map, temp, result,  TM_CCOEFF);
	minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	std::cout << "MaxVal:" << maxVal << std::endl;
	std::cout << "MaxLoc.x:" << minLoc.x << std::endl;
	std::cout << "MaxLoc.y:" << minLoc.y << std::endl;
	return maxLoc;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "beepboop"); //Ros Initialize
    ros::start();
    ros::Rate loop_rate(T); //Set Ros frequency to 50/s (fast)
    ros::NodeHandle n;

//    ros::Subscriber cameraImageLeft=n.subscribe<sensor_msgs::Image>("template", 1, template_cb);

Mat map = imread("/home/aaron/catkin_ws/src/coconuts_odroid/src/map.png", 1 );
//cvtColor(map,map,COLOR_BGR2GRAY,0);
//Canny(map,map,50,200,3,false);

Mat temp=imread("/home/aaron/catkin_ws/src/coconuts_odroid/src/template.png",1);
//cvtColor(temp,temp,COLOR_BGR2GRAY,0);
//Canny(temp,temp,50,200,3,false);

cv::Size t = temp.size();
int tH = t.height;
int tW = t.width;

cv::Size m = map.size();
int mH = m.height;
int mW = m.width;



//std::cout << "Size: " << tH <<std::endl;
//cv::imshow(WINDOW1, temp);
//cv::waitKey(10);

cv::imshow(WINDOW1, map);
cv::waitKey(1000);

cv::imshow(WINDOW2, temp);
cv::waitKey(1000);

Point matchLoc=tMatch(map,temp,mH-tH+1,mW-tW+1);
rectangle(map, matchLoc, Point( matchLoc.x + tW , matchLoc.y + tH ), Scalar::all(0), 2, 8, 0 );





    while (ros::ok()) 
    {

	ros::spinOnce();
	



        loop_rate.sleep();
    }
}

