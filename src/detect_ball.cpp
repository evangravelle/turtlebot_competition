#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "std_msgs/Int32.h"
#include "nav_msgs/Odometry.h"
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/markerdetector.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h> // getenv

// Create publishers
image_transport::Publisher pub;
ros::Publisher image_thresh_pub;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "DETECT_BALL";

// Initialize variables
ros::Time current_time;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    const sensor_msgs::ImageConstPtr hsv_image;
    current_time = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr_raw;
    cv_bridge::CvImagePtr cv_ptr_hsv;

    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    //Convert to HSV
    //cv::cvtColor(cv_ptr_raw->image, cv_ptr_hsv->image, CV_BGR2HSV);

    //Display the HSV image
    cv::imshow(WINDOW, cv_ptr_raw->image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    //pub.publish(cv_ptr->toImageMsg());

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "detect_ball");
	
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
 
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	
    pub = it.advertise("/detect_ball/hsv_image", 1);

	ros::spin();

    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
	cv::destroyWindow(WINDOW);
 }