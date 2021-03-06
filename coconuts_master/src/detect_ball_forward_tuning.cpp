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
ros::Publisher image_thresh_pub, ball_pixel_pub;
geometry_msgs::Point ball, previous_best_circle_center_orange, previous_best_circle_center_green;
coconuts_common::ControlState current_state;
float epsilon = 0.0001;
float error_threshold_orange = 0.35;
float error_threshold_green = 0.35;

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
// Initialize variables
const int max_circles = 3; // Maximum number of circles to draw
int H_MIN_ORANGE, H_MAX_ORANGE, S_MIN_ORANGE, S_MAX_ORANGE, V_MIN_ORANGE, V_MAX_ORANGE; // To be loaded from parameter server
int H_MIN_GREEN, H_MAX_GREEN, S_MIN_GREEN, S_MAX_GREEN, V_MIN_GREEN, V_MAX_GREEN; // To be loaded from parameter server

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

void on_trackbar(int,void*) {}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image) {
    // Does nothing if the state isn't in FIND_BALL
    if (true) { //current_state.state == FIND_BALL || current_state.state == MOVE_TO_BALL

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

        cv::Mat hsv_image, hsv_channels[3], hsv_thresh_orange, hsv_thresh_green;

        // Convert to HSV
        cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
        //cv::imshow(WINDOW1, hsv_image);

        // split HSV, then threshold for orange
        cv::split(hsv_image, hsv_channels);
        cv::inRange(hsv_image, cv::Scalar(H_MIN_ORANGE, S_MIN_ORANGE, V_MIN_ORANGE), cv::Scalar(H_MAX_ORANGE, S_MAX_ORANGE, V_MAX_ORANGE), hsv_thresh_orange);
        //cv::imshow(WINDOW2, hsv_thresh);

        cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

        // Erode then display
        cv::erode(hsv_thresh_orange, hsv_thresh_orange, erodeElement,cv::Point(-1,-1),2);
        // cv::imshow(WINDOW3, hsv_thresh);

        // Dilate then display
        cv::dilate(hsv_thresh_orange, hsv_thresh_orange, dilateElement);
        //cv::imshow(WINDOW4, hsv_thresh);

        // Blur image
        cv::GaussianBlur(hsv_thresh_orange, hsv_thresh_orange, cv::Size(9, 9), 2, 2);

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

        cv::Mat canny_output_orange;
        std::vector<std::vector<cv::Point> > contours_orange;
        std::vector<cv::Vec4i> hierarchy_orange;
        cv::Point2f enclosing_circle_center_orange, best_circle_center_orange;
        float enclosing_circle_radius_orange, best_circle_radius_orange;
        float contour_area_orange;

        /// Detect edges using canny
        int low_thresh = 100;
        cv::Canny(hsv_thresh_orange, canny_output_orange, low_thresh, low_thresh*2, 3 );
        /// Find contours
        cv::findContours(canny_output_orange, contours_orange, hierarchy_orange, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /// Draw contours
        cv::Mat drawing_orange = cv::Mat::zeros( canny_output_orange.size(), CV_8UC3 );

        float best_distance_orange = 2000;
        float best_error_orange = 1.0;
        float current_error_orange, current_distance_orange;
        for(int i = 0; i < contours_orange.size(); i++) {
            cv::drawContours(drawing_orange, contours_orange, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy_orange, 0, cv::Point() );
            cv::minEnclosingCircle(contours_orange[i], enclosing_circle_center_orange, enclosing_circle_radius_orange);
            contour_area_orange = cv::contourArea(contours_orange[i]);
            current_error_orange = (M_PI*pow(enclosing_circle_radius_orange,2) - contour_area_orange) / (M_PI*pow(enclosing_circle_radius_orange,2));
            current_distance_orange = calculateDistance(enclosing_circle_center_orange.x, enclosing_circle_center_orange.y, 
                previous_best_circle_center_orange.x, previous_best_circle_center_orange.y);

            // if no previous orange best ball location
            if (compareFloats(previous_best_circle_center_orange.x, -1.0, epsilon)) {
                // saves most circular, above a certain threshold, within size thresholds, and closer to previous circle
                if (current_error_orange < best_error_orange && current_error_orange < error_threshold_orange && 
                  enclosing_circle_radius_orange > min_radius && enclosing_circle_radius_orange < max_radius && 
                  (enclosing_circle_center_orange.x < 360 || enclosing_circle_center_orange.y < 240)) {
                    best_error_orange = current_error_orange;
                    best_circle_radius_orange = enclosing_circle_radius_orange;
                    best_circle_center_orange = enclosing_circle_center_orange;
                }
            }
            else {
                // saves closest to previous circle, within size thresholds, and above circle threshold
                if (current_distance_orange < best_distance_orange && enclosing_circle_radius_orange > min_radius && 
                  enclosing_circle_radius_orange < max_radius && current_error_orange < error_threshold_orange &&
                  (enclosing_circle_center_orange.x < 360 || enclosing_circle_center_orange.y < 240)) {
                    best_error_orange = current_error_orange;
                    best_distance_orange = current_distance_orange;
                    best_circle_radius_orange = enclosing_circle_radius_orange;
                    best_circle_center_orange = enclosing_circle_center_orange;
                }
            }
         }

        // if no orange ball was detected, reset previous best orange value and look for green
        if (compareFloats(best_error_orange, 1.0, epsilon)) {
            previous_best_circle_center_orange.x = -1.0;
            previous_best_circle_center_orange.y = -1.0;
            // std::cout << "no ball" << std::endl;

            cv::inRange(hsv_image, cv::Scalar(H_MIN_GREEN, S_MIN_GREEN, V_MIN_GREEN), cv::Scalar(H_MAX_GREEN, S_MAX_GREEN, V_MAX_GREEN), hsv_thresh_green);
            //cv::imshow(WINDOW2, hsv_thresh);

            // Erode then display
            cv::erode(hsv_thresh_green, hsv_thresh_green, erodeElement,cv::Point(-1,-1),2);
            // cv::imshow(WINDOW3, hsv_thresh);

            // Dilate then display
            cv::dilate(hsv_thresh_green, hsv_thresh_green, dilateElement);
            //cv::imshow(WINDOW4, hsv_thresh);

            // Blur image
            cv::GaussianBlur(hsv_thresh_green, hsv_thresh_green, cv::Size(9, 9), 2, 2);

            cv::Mat canny_output_green;
            std::vector<std::vector<cv::Point> > contours_green;
            std::vector<cv::Vec4i> hierarchy_green;
            cv::Point2f enclosing_circle_center_green, best_circle_center_green;
            float enclosing_circle_radius_green, best_circle_radius_green;
            float contour_area_green;

            /// Detect edges using canny
            cv::Canny(hsv_thresh_green, canny_output_green, low_thresh, low_thresh*2, 3 );
            /// Find contours
            cv::findContours(canny_output_green, contours_green, hierarchy_green, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

            /// Draw contours
            cv::Mat drawing_green = cv::Mat::zeros( canny_output_green.size(), CV_8UC3 );

            float best_distance_green = 2000;
            float best_error_green = 1.0;
            float current_error_green, current_distance_green;
            for(int i = 0; i < contours_green.size(); i++) {
                cv::drawContours(drawing_green, contours_green, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy_green, 0, cv::Point() );
                cv::minEnclosingCircle(contours_green[i], enclosing_circle_center_green, enclosing_circle_radius_green);
                contour_area_green = cv::contourArea(contours_green[i]);
                current_error_green = (M_PI*pow(enclosing_circle_radius_green,2) - contour_area_green) / (M_PI*pow(enclosing_circle_radius_green,2));
                current_distance_green = calculateDistance(enclosing_circle_center_green.x, enclosing_circle_center_green.y, 
                    previous_best_circle_center_green.x, previous_best_circle_center_green.y);

                // if no previous best green ball location
                if (compareFloats(previous_best_circle_center_green.x, -1.0, epsilon)) {
                    // saves most circular, above a certain threshold, within size thresholds, and closer to previous circle
                    if (current_error_green < best_error_green && current_error_green < error_threshold_green && 
                      enclosing_circle_radius_green > min_radius && enclosing_circle_radius_green < max_radius && 
                      (enclosing_circle_center_green.x < 360 || enclosing_circle_center_green.y < 240)) {
                        best_error_green = current_error_green;
                        best_circle_radius_green = enclosing_circle_radius_green;
                        best_circle_center_green = enclosing_circle_center_green;
                    }
                }
                else {
                    // saves closest to previous circle, within size thresholds, and above circle threshold
                    if (current_distance_green < best_distance_green && enclosing_circle_radius_green > min_radius && 
                      enclosing_circle_radius_green < max_radius && current_error_green < error_threshold_green &&
                      (enclosing_circle_center_green.x < 360 || enclosing_circle_center_green.y < 240)) {
                        best_error_green = current_error_green;
                        best_distance_green = current_distance_green;
                        best_circle_radius_green = enclosing_circle_radius_green;
                        best_circle_center_green = enclosing_circle_center_green;
                    }
                }
            }

            // if no green ball was detected, reset previous best green value
            if (compareFloats(best_error_green, 1.0, epsilon)) {
                previous_best_circle_center_green.x = -1.0;
                previous_best_circle_center_green.y = -1.0;
            }

            // if green ball was detected, store location
            else {
                previous_best_circle_center_green.x = best_circle_center_green.x;
                previous_best_circle_center_green.y = best_circle_center_green.y;

                // std::cout << "ball at " << previous_best_circle_center.x << ", " << previous_best_circle_center.y << std::endl;
                // std::cout << "error = " << best_error << std::endl;
                cv::circle(cv_ptr_raw->image, best_circle_center_green, best_circle_radius_green, cv::Scalar( 0, 255, 0),2);
                ball.x = best_circle_center_green.x;
                ball.y = best_circle_center_green.y;
                ball_pixel_pub.publish(ball);
            }

        }

        // if orange ball was detected, store location
        else {
            previous_best_circle_center_orange.x = best_circle_center_orange.x;
            previous_best_circle_center_orange.y = best_circle_center_orange.y;

            // std::cout << "ball at " << previous_best_circle_center.x << ", " << previous_best_circle_center.y << std::endl;
            // std::cout << "error = " << best_error << std::endl;
            cv::circle(cv_ptr_raw->image, best_circle_center_orange, best_circle_radius_orange, cv::Scalar( 255, 165, 0),2);
            ball.x = best_circle_center_orange.x;
            ball.y = best_circle_center_orange.y;
            ball_pixel_pub.publish(ball);
        }

        //cv::imshow(WINDOW1, cv_ptr_raw->image);

        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        cv::waitKey(3);

        //Convert the CvImage to a ROS image message and publish it
        it_pub.publish(cv_ptr_raw->toImageMsg());  
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "detect_ball_forward_tuning");

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

    nh.getParam("/detect_ball_forward/h_min_orange", H_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/h_max_orange", H_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/s_min_orange", S_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/s_max_orange", S_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/v_min_orange", V_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/v_max_orange", V_MAX_ORANGE);

    nh.getParam("/detect_ball_forward/h_min_green", H_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/h_max_green", H_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/s_min_green", S_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/s_max_green", S_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/v_min_green", V_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/v_max_green", V_MAX_ORANGE);

    cv::namedWindow("trackbars",0);

    cv::createTrackbar("H_MIN", "trackbars", &H_MIN, H_TOP, on_trackbar);
    cv::createTrackbar("H_MAX", "trackbars", &H_MAX, H_TOP, on_trackbar);
    cv::createTrackbar("S_MIN", "trackbars", &S_MIN, S_TOP, on_trackbar);
    cv::createTrackbar("S_MAX", "trackbars", &S_MAX, S_TOP, on_trackbar);
    cv::createTrackbar("V_MIN", "trackbars", &V_MIN, V_TOP, on_trackbar);
    cv::createTrackbar("V_MAX", "trackbars", &V_MAX, V_TOP, on_trackbar);

    previous_best_circle_center.x = -1.0;
    previous_best_circle_center.y = -1.0;

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

