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

// display images?
bool display;
bool require_correct_state;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW1[] = "/detect_ball_down_orange/image_raw";
static const char WINDOW2[] = "/detect_ball_down_orange/hsv_thresh";
//static const char WINDOW3[] = "/detect_ball_down_orange/after_erode";
//static const char WINDOW4[] = "/detect_ball_down_orange/after_dilate";
//static const char WINDOW5[] = "/detect_ball_down_orange/hsv_thresh";

// Initialize variables
image_transport::Publisher it_pub;
ros::Publisher ball_pixel_pub, control_state_pub, image_thresh_pub;
geometry_msgs::Point ball;
ros::Time current_time;
const int max_circles = 1; // Maximum number of circles to draw
int H_TOP = 179; // top end value of sliders
int S_TOP = 255;
int V_TOP = 255;
int H_MIN_ORANGE, H_MAX_ORANGE, S_MIN_ORANGE, S_MAX_ORANGE, V_MIN_ORANGE, V_MAX_ORANGE; // To be loaded from parameter server
int H_MIN_ORANGE_CHECK, H_MAX_ORANGE_CHECK, S_MIN_ORANGE_CHECK;
int S_MAX_ORANGE_CHECK, V_MIN_ORANGE_CHECK, V_MAX_ORANGE_CHECK;
coconuts_common::ControlState current_state;
float error_floor_threshold = 0.35;
float error_grab_threshold = 0.7;
float min_floor_radius = 35;
float min_grab_radius = 35;
float grab_ball_center_x = 331;
float grab_ball_center_y = 332;
float grab_ball_center_dist = 50;

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
    if (current_state.sub_state == MOVING_TO_ORANGE || current_state.sub_state == AT_ORANGE || 
      current_state.sub_state == CHECK_ORANGE || !require_correct_state) {

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
        if (current_state.sub_state == CHECK_ORANGE) {
          cv::inRange(hsv_image, cv::Scalar(H_MIN_ORANGE_CHECK, S_MIN_ORANGE_CHECK, V_MIN_ORANGE_CHECK), 
            cv::Scalar(H_MAX_ORANGE_CHECK, S_MAX_ORANGE_CHECK, V_MAX_ORANGE_CHECK), hsv_thresh);
        }
        else {
            cv::inRange(hsv_image, cv::Scalar(H_MIN_ORANGE, S_MIN_ORANGE, V_MIN_ORANGE), 
              cv::Scalar(H_MAX_ORANGE, S_MAX_ORANGE, V_MAX_ORANGE), hsv_thresh);
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
        float best_error = 1.0;
        float current_error;

        if ((current_state.sub_state == MOVING_TO_ORANGE || current_state.sub_state == AT_ORANGE || !require_correct_state) &&
          best_circle_radius > min_floor_radius) {
            for(int i = 0; i < contours.size(); i++) {
                cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
                cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
                contour_area = cv::contourArea(contours[i]);
                current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
                if (current_error < error_floor_threshold && current_error < best_error && enclosing_circle_radius > min_floor_radius) {
                    best_error = current_error;
                    best_circle_radius = enclosing_circle_radius;
                    best_circle_center = enclosing_circle_center;
                }
            }

            cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 0, 165, 255),2);
            ball.x=best_circle_center.x;
            ball.y=best_circle_center.y;
            ball_pixel_pub.publish(ball);
        }
        else if (current_state.sub_state == CHECK_ORANGE) {
            for(int i = 0; i < contours.size(); i++) {
                cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
                cv::minEnclosingCircle(contours[i], enclosing_circle_center, enclosing_circle_radius);
                contour_area = cv::contourArea(contours[i]);
                current_error = (M_PI*pow(enclosing_circle_radius,2) - contour_area) / (M_PI*pow(enclosing_circle_radius,2));
                if (current_error < error_grab_threshold && current_error < best_error && enclosing_circle_radius > min_grab_radius) {
                    best_error = current_error;
                    best_circle_radius = enclosing_circle_radius;
                    best_circle_center = enclosing_circle_center;
                }
            }

            cv::circle(cv_ptr_raw->image, best_circle_center, best_circle_radius, cv::Scalar( 0, 165, 255),2);

            if (best_circle_radius > min_grab_radius && 
              calculateDistance(best_circle_center.x, best_circle_center.y, grab_ball_center_x, grab_ball_center_y) < grab_ball_center_dist) {
                current_state.sub_state = GOT_BALL;
            }
            else {
                current_state.sub_state = GOT_BALL_FAILED;
            }

            control_state_pub.publish(current_state);
            ros::Duration(1).sleep();
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

	ros::init(argc, argv, "detect_ball_down_orange");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    if (display) {
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
        cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    }

    nh.getParam("/detect_ball_down_orange/display", display);
    nh.getParam("/detect_ball_down_orange/require_correct_state", require_correct_state);

    nh.getParam("/detect_ball_down_orange/h_min_orange", H_MIN_ORANGE);
    nh.getParam("/detect_ball_down_orange/h_max_orange", H_MAX_ORANGE);
    nh.getParam("/detect_ball_down_orange/s_min_orange", S_MIN_ORANGE);
    nh.getParam("/detect_ball_down_orange/s_max_orange", S_MAX_ORANGE);
    nh.getParam("/detect_ball_down_orange/v_min_orange", V_MIN_ORANGE);
    nh.getParam("/detect_ball_down_orange/v_max_orange", V_MAX_ORANGE);

    nh.getParam("/detect_ball_down_orange/h_min_orange_check", H_MIN_ORANGE_CHECK);
    nh.getParam("/detect_ball_down_orange/h_max_orange_check", H_MAX_ORANGE_CHECK);
    nh.getParam("/detect_ball_down_orange/s_min_orange_check", S_MIN_ORANGE_CHECK);
    nh.getParam("/detect_ball_down_orange/s_max_orange_check", S_MAX_ORANGE_CHECK);
    nh.getParam("/detect_ball_down_orange/v_min_orange_check", V_MIN_ORANGE_CHECK);
    nh.getParam("/detect_ball_down_orange/v_max_orange_check", V_MAX_ORANGE_CHECK);

    if (display) {
        cv::namedWindow("trackbars_orange",0);

        cv::createTrackbar("H_MIN", "trackbars_orange", &H_MIN_ORANGE, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_orange", &H_MAX_ORANGE, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_orange", &S_MIN_ORANGE, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_orange", &S_MAX_ORANGE, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_orange", &V_MIN_ORANGE, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_orange", &V_MAX_ORANGE, V_TOP, onTrackbar);

        cv::namedWindow("trackbars_orange_check",0);

        cv::createTrackbar("H_MIN", "trackbars_orange_check", &H_MIN_ORANGE_CHECK, H_TOP, onTrackbar);
        cv::createTrackbar("H_MAX", "trackbars_orange_check", &H_MAX_ORANGE_CHECK, H_TOP, onTrackbar);
        cv::createTrackbar("S_MIN", "trackbars_orange_check", &S_MIN_ORANGE_CHECK, S_TOP, onTrackbar);
        cv::createTrackbar("S_MAX", "trackbars_orange_check", &S_MAX_ORANGE_CHECK, S_TOP, onTrackbar);
        cv::createTrackbar("V_MIN", "trackbars_orange_check", &V_MIN_ORANGE_CHECK, V_TOP, onTrackbar);
        cv::createTrackbar("V_MAX", "trackbars_orange_check", &V_MAX_ORANGE_CHECK, V_TOP, onTrackbar);
    }

    image_transport::Subscriber sub = it.subscribe("/camera_down/image_raw", 1, imageCallback);
	ball_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_ball_down_orange/ball_pixel", 1, true);
    control_state_pub = nh.advertise<coconuts_common::ControlState>("/control_substate", 1, true);
    ros::Subscriber control_state_sub = nh.subscribe<coconuts_common::ControlState>("/control_state", 1, stateCallback);
    it_pub = it.advertise("/detect_ball_down_orange/ball_circles", 1);

	ros::spin();
    
    if (display) {
        cv::destroyWindow(WINDOW1);
        cv::destroyWindow(WINDOW2);
    }
}