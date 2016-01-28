#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
// Create publishers

//


//Changelog

//Aaron: 1/30
//Added publisher to distance finder
//Filter circles

//




image_transport::Publisher pub;
ros::Publisher image_thresh_pub;
ros::Publisher ball_location_pub;
geometry_msgs::Pose ball;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/detect_ball/image_hsv";
static const char WINDOW2[] = "/detect_ball/hsv_thresh";
static const char WINDOW3[] = "/detect_ball/after_erode";
static const char WINDOW4[] = "/detect_ball/after_dilate";
//static const char WINDOW5[] = "/detect_ball/hsv_thresh";

// Initialize variables
ros::Time current_time;
const int max_circles = 1; // Maximum number of circles to draw
int H_MIN = 0;
int H_MAX = 179;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

void on_trackbar(int,void*) {


}

void createTrackbars() {
    cv::namedWindow("trackbars",0);

    cv::createTrackbar("H_MIN", "trackbars", &H_MIN, H_MAX, on_trackbar);
    cv::createTrackbar("H_MAX", "trackbars", &H_MAX, H_MAX, on_trackbar);
    cv::createTrackbar("S_MIN", "trackbars", &S_MIN, S_MAX, on_trackbar);
    cv::createTrackbar("S_MAX", "trackbars", &S_MAX, S_MAX, on_trackbar);
    cv::createTrackbar("V_MIN", "trackbars", &V_MIN, V_MAX, on_trackbar);
    cv::createTrackbar("V_MAX", "trackbars", &V_MAX, V_MAX, on_trackbar);
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    current_time = ros::Time::now();
    // const sensor_msgs::ImageConstPtr hsv_image;
    cv_bridge::CvImagePtr cv_ptr_raw;

    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
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
    // cv::medianBlur(cv_ptr_raw->image, cv_ptr_raw->image, 3);

    // Convert to HSV
    cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
    //cv::imshow(WINDOW1, hsv_image);

    // split HSV, then threshold
    cv::split(hsv_image, hsv_channels);
    cv::inRange(hsv_image, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), hsv_thresh);
    cv::imshow(WINDOW2, hsv_thresh);

    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

    // Erode then display
    cv::erode(hsv_thresh, hsv_thresh, erodeElement,cv::Point(-1,-1),2);
    cv::imshow(WINDOW3, hsv_thresh);

    // Dilate then display
    cv::dilate(hsv_thresh, hsv_thresh, dilateElement);
    cv::imshow(WINDOW4, hsv_thresh);

    // Blur image
    cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 2, 2);

    // Use Hough tranform to search for circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(hsv_thresh, circles, CV_HOUGH_GRADIENT, 2, hsv_thresh.rows/8, 100, 20);

    if(circles.size() > 0) {
        int circles_to_draw = (circles.size() < max_circles) ? circles.size() : 1;
        for(int current_circle = 0; current_circle < circles_to_draw; ++current_circle) {
//        for(int current_circle = 0; current_circle < 1; ++current_circle) {
            cv::Point center(std::floor(circles[current_circle][0]), std::floor(circles[current_circle][1]));
            int radius = std::floor(circles[current_circle][2]);

            cv::circle(hsv_image, center, radius, cv::Scalar(0, 255, 0), 5);

		    ball.position.x=center.x;
		    ball.position.y=center.y;
		    ball_location_pub.publish(ball);

        }
    }

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
    cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
    //cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);

    createTrackbars();
 
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	ball_location_pub = nh.advertise<geometry_msgs::Pose>("/ballLocation",1000,true);
    pub = it.advertise("/detect_ball/hsv_image", 1);

	ros::spin();

    ROS_INFO("Detect_ball closed successfully");
	cv::destroyWindow(WINDOW1);
    cv::destroyWindow(WINDOW2);
    cv::destroyWindow(WINDOW3);
    cv::destroyWindow(WINDOW4);
    //cv::destroyWindow(WINDOW5);
 }
