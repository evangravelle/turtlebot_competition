#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <states.h>
#include <coconuts_common/ControlState.h>

// Create publishers
image_transport::Publisher it_pub;
ros::Publisher image_thresh_pub, control_state_pub, ball_pixel_pub;
geometry_msgs::Point ball, previous_best_circle_center_orange, previous_best_circle_center_green;
coconuts_common::ControlState current_state, orange_fail, green_fail, green_found, orange_found;
float epsilon = 0.0001;
float error_threshold_orange = 0.35;
float error_threshold_green = 0.35;
int no_orange_counter = 0;
int no_green_counter = 0;
int low_thresh = 100; // Canny edge detector
float radius_error = 3; // pixel width of allowable radius
float best_error_orange, best_error_green;
cv::Point2f best_circle_center_orange, best_circle_center_green;
float best_circle_radius_orange, best_circle_radius_green;
cv::Mat drawing_orange, drawing_green;
cv_bridge::CvImagePtr cv_ptr_raw;
bool display;
bool require_correct_state;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/usb_cam/image_rect_color";
static const char WINDOW2[] = "/detect_ball_forward/hsv_thresh_orange";
static const char WINDOW3[] = "/detect_ball_forward/contours_orange";
//static const char WINDOW3[] = "/detect_ball_forward/after_erode";
//static const char WINDOW4[] = "/detect_ball_forward/after_dilate";
static const char WINDOW4[] = "/detect_ball_forward/hsv_thresh_green";
static const char WINDOW5[] = "/detect_ball_forward/contours_green";

// Initialize variables
const int max_circles = 3; // Maximum number of circles to draw
int H_MIN_ORANGE, H_MAX_ORANGE, S_MIN_ORANGE, S_MAX_ORANGE, V_MIN_ORANGE, V_MAX_ORANGE; // To be loaded from parameter server
int H_MIN_GREEN, H_MAX_GREEN, S_MIN_GREEN, S_MAX_GREEN, V_MIN_GREEN, V_MAX_GREEN; // To be loaded from parameter server
int H_TOP = 179; // top end value of sliders
int S_TOP = 255;
int V_TOP = 255;
int min_radius = 5;
int max_radius = 30;

// This was found experimentally
float expected_radius(float pixel) {
    return 0.0415*pixel + 4.04;

}

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

void stateCallback(const coconuts_common::ControlState::ConstPtr& control_msg) {
    current_state.state = control_msg->state;
    current_state.sub_state = control_msg->sub_state;
}

void onTrackbar(int,void*) {}

cv_bridge::CvImagePtr loadImage(const sensor_msgs::ImageConstPtr& raw_image) {

    // Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    try
    {
        cv_ptr_raw = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::BGR8);
        return cv_ptr_raw;
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("detect_ball::cv_bridge exception: %s", e.what());
        // return;
    }
}

void findOrange() {

    cv::Mat hsv_image, hsv_channels[3], hsv_thresh_orange;

    // Convert to HSV
    cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
    //cv::imshow(WINDOW1, hsv_image);

    // split HSV, then threshold for orange
    cv::split(hsv_image, hsv_channels);

    cv::inRange(hsv_image, cv::Scalar(H_MIN_ORANGE, S_MIN_ORANGE, V_MIN_ORANGE), 
      cv::Scalar(H_MAX_ORANGE, S_MAX_ORANGE, V_MAX_ORANGE), hsv_thresh_orange);
    if (display) {
        cv::imshow(WINDOW2, hsv_thresh_orange);
    }

    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

    // Erode then display
    cv::erode(hsv_thresh_orange, hsv_thresh_orange, erodeElement,cv::Point(-1,-1),2);
    // cv::imshow(WINDOW3, hsv_thresh_orange);

    // Dilate then display
    cv::dilate(hsv_thresh_orange, hsv_thresh_orange, dilateElement);
    //cv::imshow(WINDOW4, hsv_thresh_orange);

    // Blur image
    cv::GaussianBlur(hsv_thresh_orange, hsv_thresh_orange, cv::Size(9, 9), 2, 2);

    cv::Mat canny_output_orange;
    std::vector<std::vector<cv::Point> > contours_orange;
    std::vector<cv::Vec4i> hierarchy_orange;
    cv::Point2f enclosing_circle_center_orange;
    float enclosing_circle_radius_orange;
    float contour_area_orange;

    /// Detect edges using canny
    cv::Canny(hsv_thresh_orange, canny_output_orange, low_thresh, low_thresh*2, 3 );
    /// Find contours
    cv::findContours(canny_output_orange, contours_orange, hierarchy_orange, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Draw contours
    drawing_orange = cv::Mat::zeros( canny_output_orange.size(), CV_8UC3 );

    float best_distance_orange = 2000;
    best_error_orange = 1.0;
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
              compareFloats(enclosing_circle_radius_orange, expected_radius(enclosing_circle_center_orange.y), radius_error) && 
              (enclosing_circle_center_orange.x < 360 || enclosing_circle_center_orange.y < 240)) {
                best_error_orange = current_error_orange;
                best_circle_radius_orange = enclosing_circle_radius_orange;
                best_circle_center_orange = enclosing_circle_center_orange;
            }
        }
        else {
            // saves closest to previous circle, within size thresholds, and above circle threshold
            if (current_distance_orange < best_distance_orange && enclosing_circle_radius_orange > min_radius && 
              compareFloats(enclosing_circle_radius_orange, expected_radius(enclosing_circle_center_orange.y), radius_error) &&
              (enclosing_circle_center_orange.x < 360 || enclosing_circle_center_orange.y < 240)) {
                best_error_orange = current_error_orange;
                best_distance_orange = current_distance_orange;
                best_circle_radius_orange = enclosing_circle_radius_orange;
                best_circle_center_orange = enclosing_circle_center_orange;
            }
        }
    }
    if (display) {
        cv::imshow(WINDOW3, drawing_orange);
    }

}

void findGreen() {

    cv::Mat hsv_image, hsv_channels[3], hsv_thresh_green;

    // Convert to HSV
    cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
    //cv::imshow(WINDOW1, hsv_image);

    // split HSV, then threshold for green
    cv::split(hsv_image, hsv_channels);

    cv::inRange(hsv_image, cv::Scalar(H_MIN_GREEN, S_MIN_GREEN, V_MIN_GREEN), 
      cv::Scalar(H_MAX_GREEN, S_MAX_GREEN, V_MAX_GREEN), hsv_thresh_green);
    if (display) {
        cv::imshow(WINDOW4, hsv_thresh_green);
    }

    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

    // Erode then display
    cv::erode(hsv_thresh_green, hsv_thresh_green, erodeElement,cv::Point(-1,-1),2);
    // cv::imshow(WINDOW3, hsv_thresh_green);

    // Dilate then display
    cv::dilate(hsv_thresh_green, hsv_thresh_green, dilateElement);
    //cv::imshow(WINDOW4, hsv_thresh_green);

    // Blur image
    cv::GaussianBlur(hsv_thresh_green, hsv_thresh_green, cv::Size(9, 9), 2, 2);

    cv::Mat canny_output_green;
    std::vector<std::vector<cv::Point> > contours_green;
    std::vector<cv::Vec4i> hierarchy_green;
    cv::Point2f enclosing_circle_center_green;
    float enclosing_circle_radius_green;
    float contour_area_green;

    /// Detect edges using canny
    cv::Canny(hsv_thresh_green, canny_output_green, low_thresh, low_thresh*2, 3 );
    /// Find contours
    cv::findContours(canny_output_green, contours_green, hierarchy_green, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Draw contours
    cv::Mat drawing_green = cv::Mat::zeros( canny_output_green.size(), CV_8UC3 );

    float best_distance_green = 2000;
    best_error_green = 1.0;
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
              compareFloats(enclosing_circle_radius_green, expected_radius(enclosing_circle_center_green.y), radius_error) && 
              (enclosing_circle_center_green.x < 360 || enclosing_circle_center_green.y < 240)) {
                best_error_green = current_error_green;
                best_circle_radius_green = enclosing_circle_radius_green;
                best_circle_center_green = enclosing_circle_center_green;
            }
        }
        else {
            // saves closest to previous circle, within size thresholds, and above circle threshold
            if (current_distance_green < best_distance_green && enclosing_circle_radius_green > min_radius && 
              compareFloats(enclosing_circle_radius_green, expected_radius(enclosing_circle_center_green.y), radius_error) &&
              (enclosing_circle_center_green.x < 360 || enclosing_circle_center_green.y < 240)) {
                best_error_green = current_error_green;
                best_distance_green = current_distance_green;
                best_circle_radius_green = enclosing_circle_radius_green;
                best_circle_center_green = enclosing_circle_center_green;
            }
        }
    }

    if (display) {
        cv::imshow(WINDOW5, drawing_green);
    }
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image) {

    // If state is FIND_BALL, look for orange over green
    if (current_state.state == FIND_BALL || !require_correct_state) {

        cv_ptr_raw = loadImage(raw_image);
        findOrange();

        // if orange ball was detected, store location
        if (!compareFloats(best_error_orange, 1.0, epsilon)) {

            previous_best_circle_center_orange.x = best_circle_center_orange.x;
            previous_best_circle_center_orange.y = best_circle_center_orange.y;

            std::cout << "orange ball at " << previous_best_circle_center_orange.x << ", " << previous_best_circle_center_orange.y << std::endl;
            std::cout << "orange error = " << best_error_orange << std::endl;
            std::cout << "orange radius = " << best_circle_radius_orange << std::endl;
            cv::circle(cv_ptr_raw->image, best_circle_center_orange, best_circle_radius_orange, cv::Scalar( 0, 165, 255),2);
            ball.x = best_circle_center_orange.x;
            ball.y = best_circle_center_orange.y;
            ball_pixel_pub.publish(ball);

            std::cout << "state changed to ORANGE_BALL_FOUND" << std::endl;
            orange_found.state = MOVE_TO_BALL;
	    orange_found.sub_state= MOVING_TO_ORANGE;
            control_state_pub.publish(orange_found);
        }

        // if no orange ball was detected, reset previous best orange value and look for green   
        else {

            previous_best_circle_center_orange.x = -1.0;
            previous_best_circle_center_orange.y = -1.0;
            // std::cout << "no ball" << std::endl;
            findGreen();

            // if no green ball was detected, reset previous best green value
            if (compareFloats(best_error_green, 1.0, epsilon)) {
                previous_best_circle_center_green.x = -1.0;
                previous_best_circle_center_green.y = -1.0;
            }

            // if green ball was detected, store location
            else {
                previous_best_circle_center_green.x = best_circle_center_green.x;
                previous_best_circle_center_green.y = best_circle_center_green.y;

                std::cout << "green ball at " << previous_best_circle_center_green.x << ", " << previous_best_circle_center_green.y << std::endl;
                std::cout << "green error = " << best_error_green << std::endl;
                std::cout << "green radius = " << best_circle_radius_green << std::endl;
                cv::circle(cv_ptr_raw->image, best_circle_center_green, best_circle_radius_green, cv::Scalar( 0, 255, 0),2);
                ball.x = best_circle_center_green.x;
                ball.y = best_circle_center_green.y;
                ball_pixel_pub.publish(ball);

                std::cout << "state changed to GREEN_BALL_FOUND" << std::endl;
                green_found.state = MOVE_TO_BALL;
		green_found.sub_state = MOVING_TO_GREEN;
                control_state_pub.publish(green_found);
            }
        }

        if (display) {
            cv::imshow(WINDOW1, cv_ptr_raw->image);
        }

        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        cv::waitKey(3);

        //Convert the CvImage to a ROS image message and publish it
        it_pub.publish(cv_ptr_raw->toImageMsg());  
    }

    // If sub_state is MOVING_TO_ORANGE, look for orange
    else if( (current_state.state == FIND_BALL || !require_correct_state) && current_state.sub_state == MOVING_TO_ORANGE) {

        cv_ptr_raw = loadImage(raw_image);
        findOrange();

        // if orange ball was detected, store location
        if (!compareFloats(best_error_orange, 1.0, epsilon)) {

            previous_best_circle_center_orange.x = best_circle_center_orange.x;
            previous_best_circle_center_orange.y = best_circle_center_orange.y;
            no_orange_counter = 0;

            std::cout << "orange ball at (" << previous_best_circle_center_orange.x << ", " << previous_best_circle_center_orange.y << ")" << std::endl;
            std::cout << "orange error = " << best_error_orange << std::endl;
            cv::circle(cv_ptr_raw->image, best_circle_center_orange, best_circle_radius_orange, cv::Scalar( 0, 165, 255),2);
            ball.x = best_circle_center_orange.x;
            ball.y = best_circle_center_orange.y;
            ball_pixel_pub.publish(ball);

        }

        // if orange ball is lost for 5 frames in a row, the publish failure
        else {
            no_orange_counter++;
            if (no_orange_counter > 5) {
                std::cout << "orange ball lost!" << std::endl;
                orange_fail.state = MOVE_TO_BALL;
                orange_fail.sub_state = MOVE_TO_ORANGE_FAILED;
                control_state_pub.publish(orange_fail);
            }

        }

    }

    // If sub_state is MOVING_TO_GREEN, look for green
    else if( (current_state.state == FIND_BALL || !require_correct_state) && current_state.sub_state == MOVING_TO_GREEN) {
        cv_ptr_raw = loadImage(raw_image);
        findGreen();

        // if green ball was detected, store location
        if (!compareFloats(best_error_green, 1.0, epsilon)) {

            previous_best_circle_center_green.x = best_circle_center_green.x;
            previous_best_circle_center_green.y = best_circle_center_green.y;
            no_green_counter = 0;

            std::cout << "green ball at (" << previous_best_circle_center_green.x << ", " << previous_best_circle_center_green.y << ")" << std::endl;
            std::cout << "green error = " << best_error_green << std::endl;
            cv::circle(cv_ptr_raw->image, best_circle_center_green, best_circle_radius_green, cv::Scalar( 0, 165, 255),2);
            ball.x = best_circle_center_green.x;
            ball.y = best_circle_center_green.y;
            ball_pixel_pub.publish(ball);

        }

        // if green ball is lost for 5 frames in a row, the publish failure
        else {
            no_green_counter++;
            if (no_orange_counter > 5) {
                std::cout << "green ball lost!" << std::endl;
                green_fail.state = MOVE_TO_BALL;
                green_fail.sub_state = MOVE_TO_GREEN_FAILED;
                control_state_pub.publish(green_fail);
            }

        }

    }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "detect_ball_forward");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    if (display) {
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
        cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW3, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW4, CV_WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW5, CV_WINDOW_AUTOSIZE);
    }

    nh.getParam("/detect_ball_forward/display", display);
    nh.getParam("/detect_ball_forward/require_correct_state", require_correct_state);

    nh.getParam("/detect_ball_forward/h_min_orange", H_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/h_max_orange", H_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/s_min_orange", S_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/s_max_orange", S_MAX_ORANGE);
    nh.getParam("/detect_ball_forward/v_min_orange", V_MIN_ORANGE);
    nh.getParam("/detect_ball_forward/v_max_orange", V_MAX_ORANGE);

    nh.getParam("/detect_ball_forward/h_min_green", H_MIN_GREEN);
    nh.getParam("/detect_ball_forward/h_max_green", H_MAX_GREEN);
    nh.getParam("/detect_ball_forward/s_min_green", S_MIN_GREEN);
    nh.getParam("/detect_ball_forward/s_max_green", S_MAX_GREEN);
    nh.getParam("/detect_ball_forward/v_min_green", V_MIN_GREEN);
    nh.getParam("/detect_ball_forward/v_max_green", V_MAX_GREEN);

    if (display) {
        cv::namedWindow("trackbars_orange",0);
        cv::namedWindow("trackbars_green",0);

        cv::createTrackbar("H_MIN", "trackbars_orange", &H_MIN_ORANGE, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_orange", &H_MAX_ORANGE, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_orange", &S_MIN_ORANGE, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_orange", &S_MAX_ORANGE, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_orange", &V_MIN_ORANGE, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_orange", &V_MAX_ORANGE, V_TOP, onTrackbar);


        cv::createTrackbar("H_MIN", "trackbars_green", &H_MIN_GREEN, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_green", &H_MAX_GREEN, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_green", &S_MIN_GREEN, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_green", &S_MAX_GREEN, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_green", &V_MIN_GREEN, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_green", &V_MAX_GREEN, V_TOP, onTrackbar);
    }

    //image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/camera_forward/image_rect_color", 1, imageCallback);
	ball_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_ball_forward/ball_pixel",1,true);
    ros::Subscriber control_state_sub = nh.subscribe<coconuts_common::ControlState>("/control_state", 1, stateCallback);
    control_state_pub = nh.advertise<coconuts_common::ControlState>("/control_substate", 1, true);
    it_pub = it.advertise("/detect_ball_forward/ball_circles", 1);

    previous_best_circle_center_orange.x = -1.0;
    previous_best_circle_center_orange.y = -1.0;
    previous_best_circle_center_green.x = -1.0;
    previous_best_circle_center_green.y = -1.0;
    float best_error_orange = 1.0;
    float best_error_green = 1.0;

	ros::spin();

    //nh.setParam("/detect_ball_forward/h_min", H_MIN);
    //nh.setParam("/detect_ball_forward/h_max", H_MAX);
    //nh.setParam("/detect_ball_forward/s_min", S_MIN);
    //nh.setParam("/detect_ball_forward/s_max", S_MAX);
    //nh.setParam("/detect_ball_forward/v_min", V_MIN);
    //nh.setParam("/detect_ball_forward/v_max", V_MAX);
    if (display) {
        cv::destroyWindow(WINDOW1);
        cv::destroyWindow(WINDOW2);
        cv::destroyWindow(WINDOW3);
        cv::destroyWindow(WINDOW4);
        cv::destroyWindow(WINDOW5);
    }

 }






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
