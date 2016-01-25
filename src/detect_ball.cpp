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

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/detect_ball/image_hsv";
//static const char WINDOW2[] = "/detect_ball/hue";
//static const char WINDOW3[] = "/detect_ball/sat";
//static const char WINDOW4[] = "/detect_ball/val";
static const char WINDOW5[] = "/detect_ball/hsv_thresh";

// Initialize variables
ros::Time current_time;
int max_circles = 1; // Maximum number of circles to draw


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    current_time = ros::Time::now();
    // const sensor_msgs::ImageConstPtr hsv_image;
    cv_bridge::CvImagePtr cv_ptr_raw;

    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("detect_ball::cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat hsv_image, hsv_channels[3], hsv_thresh;

    // remove some noise
    //cv::medianBlur(cv_ptr_raw->image, cv_ptr_raw->image, 3);

    //Convert to HSV, split HSV, then threshold
    cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
    cv::split(hsv_image, hsv_channels);
    cv::inRange(hsv_image, cv::Scalar(3, 50, 100), cv::Scalar(14, 204, 255), hsv_thresh);

    // Blur image
    cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 2, 2);

    // Use Hough tranform to search for circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(hsv_thresh, circles, CV_HOUGH_GRADIENT, 2, hsv_thresh.rows/8, 100, 20);

    if(circles.size() > 0) {
        int circles_to_draw = (circles.size() < max_circles) ? circles.size() : 5;
        for(int current_circle = 0; current_circle < circles_to_draw; ++current_circle) {
            cv::Point center(std::floor(circles[current_circle][0]), std::floor(circles[current_circle][1]));
            int radius = std::floor(circles[current_circle][2]);

            cv::circle(hsv_image, center, radius, cv::Scalar(0, 255, 0), 5);
        }
    }

    //Display the thresholded HSV image
    cv::imshow(WINDOW1, hsv_image);
    //cv::imshow(WINDOW2, hsv_channels[1]);
    //cv::imshow(WINDOW3, hsv_channels[2]);
    //cv::imshow(WINDOW4, hsv_channels[3]);
    cv::imshow(WINDOW5, hsv_thresh);

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

    cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
    //cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    //cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
    //cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);
 
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	
    pub = it.advertise("/detect_ball/hsv_image", 1);

	ros::spin();

    ROS_INFO("Detect_ball closed successfully");
	cv::destroyWindow(WINDOW1);
    //cv::destroyWindow(WINDOW2);
    //cv::destroyWindow(WINDOW3);
    //cv::destroyWindow(WINDOW4);
    cv::destroyWindow(WINDOW5);
 }