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
geometry_msgs::Point ball, previous_best_circle_center;
float epsilon = 0.0001;
float error_threshold = 0.35;

// Initialize variables
const int max_circles = 3; // Maximum number of circles to draw
int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX; // To be loaded from parameter server
int min_radius = 5;
int max_radius = 30;

bool compareFloats(float a, float b, float epsilon) {
   float diff = a - b;
   return (diff < epsilon) && (-diff < epsilon);
}

float calculateDistance(const float x1, const float y1, const float x2, const float y2)
{
    float diffY = y2 - y1;
    float diffX = x2 - x1;
    return sqrt((diffY * diffY) + (diffX * diffX));
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{

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
    //cv::imshow(WINDOW2, hsv_thresh);

    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

    // Erode then display
    cv::erode(hsv_thresh, hsv_thresh, erodeElement,cv::Point(-1,-1),2);
//    cv::imshow(WINDOW3, hsv_thresh);

    // Dilate then display
    cv::dilate(hsv_thresh, hsv_thresh, dilateElement);
    //cv::imshow(WINDOW4, hsv_thresh);

    // Blur image
    cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 2, 2);

/*
    // Use Hough tranform to search for circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(hsv_thresh, circles, CV_HOUGH_GRADIENT, 2, hsv_thresh.rows/8, 100, 20, 10, 60);

    if(circles.size() > 0) {
        int circles_to_draw = (circles.size() < max_circles) ? circles.size() : 1;
        for(int current_circle = 0; current_circle < circles_to_draw; ++current_circle) {
//        for(int current_circle = 0; current_circle < 1; ++current_circle) {
            cv::Point center(std::floor(circles[current_circle][0]), std::floor(circles[current_circle][1]));
            int radius = std::floor(circles[current_circle][2]);

            cv::circle(cv_ptr_raw->image, center, radius, cv::Scalar(0, 255, 0), 5);

		    ball.x=center.x;
		    ball.y=center.y;
		    ball_pixel_pub.publish(ball);

        }
    }
*/

 cv::Mat canny_output;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Point2f enclosing_circle_center, best_circle_center;
    float enclosing_circle_radius, best_circle_radius;
    float contour_area;

    /// Detect edges using canny
    int low_thresh = 100;
    cv::Canny(hsv_thresh, canny_output, low_thresh, low_thresh*2, 3 );
    /// Find contours
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Draw contours
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

    float best_distance = 2000;
    float best_error = 1.0;
    float current_error, current_distance;
    for(int i = 0; i < contours.size(); i++) {
        cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
        cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
        contour_area = cv::contourArea(contours[i]);
        current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
        current_distance = calculateDistance(enclosing_circle_center.x, enclosing_circle_center.y, 
            previous_best_circle_center.x, previous_best_circle_center.y);

        // if no previous best ball location
        if (compareFloats(previous_best_circle_center.x, -1.0, epsilon)) {
            // saves most circular, above a certain threshold, within size thresholds, and closer to previous circle
            if (current_error < best_error && current_error < error_threshold && enclosing_circle_radius > min_radius && 
              enclosing_circle_radius < max_radius && (enclosing_circle_center.x < 360 || enclosing_circle_center.y < 240)) {
                best_error = current_error;
                best_circle_radius = enclosing_circle_radius;
                best_circle_center = enclosing_circle_center;
            }
        }
        else {
            // saves closest to previous circle, within size thresholds, and above circle threshold
            if (current_distance < best_distance && enclosing_circle_radius > min_radius && 
              enclosing_circle_radius < max_radius && current_error < error_threshold &&
              (enclosing_circle_center.x < 360 || enclosing_circle_center.y < 240)) {
                best_error = current_error;
                best_distance = current_distance;
                best_circle_radius = enclosing_circle_radius;
                best_circle_center = enclosing_circle_center;
            }
        }
     }

    // if no ball was detected, reset previous best value
    if (compareFloats(best_error, 1.0, epsilon)) {
        previous_best_circle_center.x = -1.0;
        previous_best_circle_center.y = -1.0;
        // std::cout << "no ball" << std::endl;
    }

    // if ball was detected, store location
    else {
        previous_best_circle_center.x = best_circle_center.x;
        previous_best_circle_center.y = best_circle_center.y;

        // std::cout << "ball at " << previous_best_circle_center.x << ", " << previous_best_circle_center.y << std::endl;
        // std::cout << "error = " << best_error << std::endl;
        cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 255, 255, 0),2);
        ball.x = best_circle_center.x;
        ball.y = best_circle_center.y;
        ball_pixel_pub.publish(ball);
    }

    //cv::imshow(WINDOW1, cv_ptr_raw->image);

    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message and publish it
    it_pub.publish(cv_ptr_raw->toImageMsg());  
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "detect_ball_forward");
	
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    nh.getParam("/detect_ball_forward/h_min", H_MIN);
    nh.getParam("/detect_ball_forward/h_max", H_MAX);
    nh.getParam("/detect_ball_forward/s_min", S_MIN);
    nh.getParam("/detect_ball_forward/s_max", S_MAX);
    nh.getParam("/detect_ball_forward/v_min", V_MIN);
    nh.getParam("/detect_ball_forward/v_max", V_MAX);

    //image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/camera_forward/image_rect_color", 1, imageCallback);
	ball_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1,true);
    it_pub = it.advertise("/detect_ball_forward/ball_circles", 1);

    previous_best_circle_center.x = -1.0;
    previous_best_circle_center.y = -1.0;

	ros::spin();

    //cv::destroyWindow(WINDOW2);
    //cv::destroyWindow(WINDOW4);

 }
