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
#include <cmath>

// display images?
bool display;
bool require_correct_state;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/detect_ball_down_green/image_raw";
static const char WINDOW2[] = "/detect_ball_down_green/hsv_thresh";
//static const char WINDOW3[] = "/detect_ball_down_green/after_erode";
//static const char WINDOW4[] = "/detect_ball_down_green/after_dilate";
//static const char WINDOW5[] = "/detect_ball_down_green/hsv_thresh";

// Initialize variables
image_transport::Publisher it_pub;
ros::Publisher ball_pixel_pub, control_state_pub, image_thresh_pub;
geometry_msgs::Point ball;
ros::Time current_time;
const int max_circles = 1; // Maximum number of circles to draw
int H_TOP = 179; // top end value of sliders
int S_TOP = 255;
int V_TOP = 255;
int H_MIN_GREEN, H_MAX_GREEN, S_MIN_GREEN, S_MAX_GREEN, V_MIN_GREEN, V_MAX_GREEN; // To be loaded from parameter server
int H_MIN_GREEN_CHECK, H_MAX_GREEN_CHECK, S_MIN_GREEN_CHECK;
int S_MAX_GREEN_CHECK, V_MIN_GREEN_CHECK, V_MAX_GREEN_CHECK;
coconuts_common::ControlState current_state, pub_state;
double t = 0;
float error_floor_threshold = 0.35;
float error_grab_threshold = 0.6;
float min_floor_radius;
float min_grab_radius;
float grab_ball_center_x;
float grab_ball_center_y;
float grab_ball_center_dist;
int image_width, image_height;

void onTrackbar(int,void*) {}

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

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    
    if (current_state.sub_state == MOVING_TO_GREEN || current_state.sub_state == AT_GREEN || current_state.sub_state == CENTER_ON_GREEN || 
      current_state.sub_state == CHECK_GREEN || !require_correct_state) {

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
        if (current_state.sub_state == CHECK_GREEN) {
          cv::inRange(hsv_image, cv::Scalar(H_MIN_GREEN_CHECK, S_MIN_GREEN_CHECK, V_MIN_GREEN_CHECK), 
            cv::Scalar(H_MAX_GREEN_CHECK, S_MAX_GREEN_CHECK, V_MAX_GREEN_CHECK), hsv_thresh);
        }
        else {
            cv::inRange(hsv_image, cv::Scalar(H_MIN_GREEN, S_MIN_GREEN, V_MIN_GREEN), 
              cv::Scalar(H_MAX_GREEN, S_MAX_GREEN, V_MAX_GREEN), hsv_thresh);
        }

        if (display) {
            cv::imshow(WINDOW2, hsv_thresh);
        }

        cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

        // Erode then display
        cv::erode(hsv_thresh, hsv_thresh, erodeElement,cv::Point(-1,-1),2);
        //cv::imshow(WINDOW3, hsv_thresh);

        // Dilate then display
        cv::dilate(hsv_thresh, hsv_thresh, dilateElement);
        //cv::imshow(WINDOW4, hsv_thresh);

        // Blur image
        cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 2, 2);
        /*
        // Use Hough tranform to search for circles
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(cv_ptr_raw->image, circles, CV_HOUGH_GRADIENT, 2, cv_ptr_raw->image.rows/8, 100, 20);

        if(circles.size() > 0) {
            int circles_to_draw = (circles.size() < max_circles) ? circles.size() : 1;
            for(int current_circle = 0; current_circle < circles_to_draw; ++current_circle) {
            //for(int current_circle = 0; current_circle < 1; ++current_circle) {
                cv::Point center(std::floor(circles[current_circle][0]), std::floor(circles[current_circle][1]));
                int radius = std::floor(circles[current_circle][2]);

                cv::circle(cv_ptr_raw->image, center, radius, cv::Scalar(0, 255, 0), 5);

              ball.position.x=center.x;
              ball.position.y=center.y;
              ball_location_pub.publish(ball);

            }
        }
        */

        // Contour method
        cv::Mat canny_output;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Point2f enclosing_circle_center;
        float enclosing_circle_radius;
        double contour_area;

        // Detect edges using canny
        int low_thresh = 100;
        cv::Canny(hsv_thresh, canny_output, low_thresh, low_thresh*2, 3 );
        // Find contours
        cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        // Draw contours
        cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );

        cv::Point2f best_circle_center;
        float best_circle_radius = 2000.0;
        float best_distance = 1000;
        float best_error = 1.0;
        float current_error, current_distance;

        if (current_state.sub_state == MOVING_TO_GREEN || current_state.sub_state == AT_GREEN || 
          current_state.sub_state == CENTER_ON_GREEN || !require_correct_state) {
            for(int i = 0; i < contours.size(); i++) {
                cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
                cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
                contour_area = cv::contourArea(contours[i]);
                current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
                if (current_error < best_error && enclosing_circle_radius > min_floor_radius) {
                    best_error = current_error;
                    best_circle_radius = enclosing_circle_radius;
                    best_circle_center = enclosing_circle_center;
                }
            }

            if (best_error < error_floor_threshold) {
                cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 0, 165, 255),2);
                ball.x = best_circle_center.x;
                ball.y = best_circle_center.y;
                ball_pixel_pub.publish(ball);
            }
            else {
                ball.x = -1;
                ball.y = -1;
                ball_pixel_pub.publish(ball);
            }
        }

        else if (current_state.sub_state == CHECK_GREEN) {
            //for(int i = 0; i < contours.size(); i++) {
            //    cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
            //    cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
            //    contour_area = cv::contourArea(contours[i]);
            //    current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
            //    current_distance = calculateDistance(enclosing_circle_center.x, enclosing_circle_center.y, grab_ball_center_x, grab_ball_center_y);
            //    if (enclosing_circle_radius > min_grab_radius && current_distance < best_distance) {
            //        best_error = current_error;
            //        best_distance = current_distance;
            //        best_circle_radius = enclosing_circle_radius;
            //        best_circle_center = enclosing_circle_center;
            //    }
            //}

            // If at least 3 seconds have passed since last publication
            if (t < 0.0001 || ros::Time::now().toSec() - t > 3) {
                //if (best_error < error_grab_threshold && best_distance < grab_ball_center_dist) {
                //    cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 0, 255, 0),2);
                    pub_state.state = PICK_UP_BALL;
                    pub_state.sub_state = GOT_BALL;
                    control_state_pub.publish(pub_state);
                    t = ros::Time::now().toSec();
                }
                //else {
                //    pub_state.state = PICK_UP_BALL;
                //    pub_state.sub_state = GOT_BALL_FAILED;
                //    control_state_pub.publish(pub_state);
                //    t = ros::Time::now().toSec();
                //}
            //}
        }

        if (display) {
            cv::imshow(WINDOW1, cv_ptr_raw->image);
        }

        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        cv::waitKey(3);

        //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        it_pub.publish(cv_ptr_raw->toImageMsg());
    }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "detect_ball_down_green");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    if (display) {
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
        cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    }

    nh.getParam("/detect_ball_down_green/display", display);
    nh.getParam("/detect_ball_down_green/require_correct_state", require_correct_state);

    nh.getParam("/detect_ball_down_green/image_width", image_width);
    nh.getParam("/detect_ball_down_green/image_height", image_height);

    nh.getParam("/detect_ball_down_green/h_min_green", H_MIN_GREEN);
    nh.getParam("/detect_ball_down_green/h_max_green", H_MAX_GREEN);
    nh.getParam("/detect_ball_down_green/s_min_green", S_MIN_GREEN);
    nh.getParam("/detect_ball_down_green/s_max_green", S_MAX_GREEN);
    nh.getParam("/detect_ball_down_green/v_min_green", V_MIN_GREEN);
    nh.getParam("/detect_ball_down_green/v_max_green", V_MAX_GREEN);

    nh.getParam("/detect_ball_down_green/h_min_green_check", H_MIN_GREEN_CHECK);
    nh.getParam("/detect_ball_down_green/h_max_green_check", H_MAX_GREEN_CHECK);
    nh.getParam("/detect_ball_down_green/s_min_green_check", S_MIN_GREEN_CHECK);
    nh.getParam("/detect_ball_down_green/s_max_green_check", S_MAX_GREEN_CHECK);
    nh.getParam("/detect_ball_down_green/v_min_green_check", V_MIN_GREEN_CHECK);
    nh.getParam("/detect_ball_down_green/v_max_green_check", V_MAX_GREEN_CHECK);

    if (display) {
        cv::namedWindow("trackbars_green",0);

        cv::createTrackbar("H_MIN", "trackbars_green", &H_MIN_GREEN, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_green", &H_MAX_GREEN, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_green", &S_MIN_GREEN, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_green", &S_MAX_GREEN, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_green", &V_MIN_GREEN, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_green", &V_MAX_GREEN, V_TOP, onTrackbar);

        cv::namedWindow("trackbars_green_check",0);

        cv::createTrackbar("H_MIN", "trackbars_green_check", &H_MIN_GREEN_CHECK, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_green_check", &H_MAX_GREEN_CHECK, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_green_check", &S_MIN_GREEN_CHECK, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_green_check", &S_MAX_GREEN_CHECK, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_green_check", &V_MIN_GREEN_CHECK, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_green_check", &V_MAX_GREEN_CHECK, V_TOP, onTrackbar);
    }

    image_transport::Subscriber sub = it.subscribe("/camera_down/image_raw", 1, imageCallback);
	ball_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_ball_down/ball_pixel", 1, true);
    control_state_pub = nh.advertise<coconuts_common::ControlState>("/control_substate", 1, true);
    ros::Subscriber control_state_sub = nh.subscribe<coconuts_common::ControlState>("/control_state", 1, stateCallback);
    it_pub = it.advertise("/detect_ball_down/ball_circles", 1);

    grab_ball_center_x = 0.5*image_width;
    grab_ball_center_y = 0.79*image_height;
    min_floor_radius = 0.055*image_width;
    min_grab_radius = 0.055*image_width;
    grab_ball_center_dist = 0.05*image_width;

	ros::spin();

    if (display) {
        cv::destroyWindow(WINDOW1);
        cv::destroyWindow(WINDOW2);
    }
}