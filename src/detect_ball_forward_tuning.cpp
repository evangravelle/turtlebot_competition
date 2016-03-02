#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>

// Create publishers
image_transport::Publisher it_pub;
ros::Publisher image_thresh_pub;

ros::Publisher ball_pixel_pub;
geometry_msgs::Point ball, biggest_circle_center;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/detect_ball_forward/image_raw";
static const char WINDOW2[] = "/detect_ball_forward/hsv_thresh";
static const char WINDOW3[] = "/detect_ball_forward/after_erode";
static const char WINDOW4[] = "/detect_ball_forward/after_dilate";
static const char WINDOW5[] = "/detect_ball_forward/contours";

// Initialize variables
ros::Time current_time;
const int max_circles = 3; // Maximum number of circles to draw
int H_TOP = 179; // top end value of sliders
int S_TOP = 255;
int V_TOP = 255;
int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX;

void on_trackbar(int,void*) {}

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

    // Blur image
    cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 2, 2);
    cv::imshow(WINDOW4, hsv_thresh);

/*
    // Use Hough tranform to search for circles
    std::vector<cv::Vec3f> circles;
    int min_radius = 5;
    int max_radius = 30;
    int accumulator_threshold = 20; // increase for less circles
    int grad_value = 100;
    cv::HoughCircles(hsv_thresh, circles, CV_HOUGH_GRADIENT, 2, 2*min_radius, grad_value, accumulator_threshold, min_radius, max_radius);

    int biggest_circle_radius = 0;
    if(circles.size() > 0) {
        int circles_to_draw = (circles.size() < max_circles) ? circles.size() : 1;
        for(int current_circle = 0; current_circle < circles_to_draw; ++current_circle) {

            cv::Point center(std::floor(circles[current_circle][0]), std::floor(circles[current_circle][1]));
            int radius = std::floor(circles[current_circle][2]);
            cv::circle(cv_ptr_raw->image, center, radius, cv::Scalar(0, 255, 0), 5);

            if (radius > biggest_circle_radius) {
                biggest_circle_radius = radius;
                biggest_circle_center.x = circles[current_circle][0];
                biggest_circle_center.y = circles[current_circle][1];
            }

        }

        ball.x=biggest_circle_center.x;
        ball.y=biggest_circle_center.y;
        ball_pixel_pub.publish(ball);
    }
*/

    cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Point2f enclosing_circle_center;
    float enclosing_circle_radius;
    double contour_area;

    /// Detect edges using canny
    int low_thresh = 100;
    cv::Canny(hsv_thresh, canny_output, low_thresh, low_thresh*2, 3 );
    /// Find contours
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

    cv::Point2f best_circle_center;
    double best_circle_radius;
    double best_error = 1.0;
    double current_error;
    for(int i = 0; i < contours.size(); i++) {
        cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
        cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
        contour_area = cv::contourArea(contours[i]);
        current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
        if (current_error < best_error) {
            best_error = current_error;
            best_circle_radius = enclosing_circle_radius;
            best_circle_center = enclosing_circle_center;
        }
     }

    cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 255, 255, 0),2);

    cv::imshow(WINDOW5, drawing);

    cv::imshow(WINDOW1, cv_ptr_raw->image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
    it_pub.publish(cv_ptr_raw->toImageMsg());

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detect_ball_tuning");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    //cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
    cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);

    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    //image_transport::Subscriber sub = it.subscribe("/stereo/right/image_rect_color", 1, imageCallback); //Testing
	ball_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1000,true);
    it_pub = it.advertise("/detect_ball_forward/ball_circles", 1);

    nh.getParam("/detect_ball_forward/h_min", H_MIN);
    nh.getParam("/detect_ball_forward/h_max", H_MAX);
    nh.getParam("/detect_ball_forward/s_min", S_MIN);
    nh.getParam("/detect_ball_forward/s_max", S_MAX);
    nh.getParam("/detect_ball_forward/v_min", V_MIN);
    nh.getParam("/detect_ball_forward/v_max", V_MAX);

    cv::namedWindow("trackbars",0);

    cv::createTrackbar("H_MIN", "trackbars", &H_MIN, H_TOP, on_trackbar);
    cv::createTrackbar("H_MAX", "trackbars", &H_MAX, H_TOP, on_trackbar);
    cv::createTrackbar("S_MIN", "trackbars", &S_MIN, S_TOP, on_trackbar);
    cv::createTrackbar("S_MAX", "trackbars", &S_MAX, S_TOP, on_trackbar);
    cv::createTrackbar("V_MIN", "trackbars", &V_MIN, V_TOP, on_trackbar);
    cv::createTrackbar("V_MAX", "trackbars", &V_MAX, V_TOP, on_trackbar);

    while(ros::ok()) {
	   ros::spin();
    }

    nh.setParam("/detect_ball_forward/h_min", H_MIN);
    nh.setParam("/detect_ball_forward/h_max", H_MAX);
    nh.setParam("/detect_ball_forward/s_min", S_MIN);
    nh.setParam("/detect_ball_forward/s_max", S_MAX);
    nh.setParam("/detect_ball_forward/v_min", V_MIN);
    nh.setParam("/detect_ball_forward/v_max", V_MAX);

	//cv::destroyWindow(WINDOW1);
    cv::destroyWindow(WINDOW2);
    cv::destroyWindow(WINDOW3);
    cv::destroyWindow(WINDOW4);
    cv::destroyWindow(WINDOW5);

 }

