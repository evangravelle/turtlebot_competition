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

static const char WINDOW1[] = "/detect_bucket_forward/image_raw";
static const char WINDOW2[] = "/detect_bucket_forward/hsv_thresh";

// Initialize variables
int H_TOP = 179; // top end value of sliders
int S_TOP = 255;
int V_TOP = 255;
int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX; // To be loaded from parameter server
image_transport::Publisher it_pub;
ros::Publisher image_thresh_pub, bucket_pixel_pub;
geometry_msgs::Point bucket;
coconuts_common::ControlState current_state;
bool display;
bool require_correct_state;

void on_trackbar(int,void*) {}

void stateCallback(const coconuts_common::ControlState::ConstPtr& control_msg) {
    current_state.state = control_msg->state;
    current_state.sub_state = control_msg->sub_state;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& raw_image)
{
    if (current_state.state == FIND_GOAL || !require_correct_state) {
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
            ROS_ERROR("detect_bucket::cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv_image, hsv_channels[3], hsv_thresh;

        // Convert to HSV
        cv::cvtColor(cv_ptr_raw->image, hsv_image, CV_BGR2HSV);
        //cv::imshow(WINDOW1, hsv_image);

        // split HSV, then threshold
        cv::split(hsv_image, hsv_channels);
        cv::inRange(hsv_image, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), hsv_thresh);
        if (display) {
            cv::imshow(WINDOW2, hsv_thresh);
        }

        cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
        cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8));

        // Erode then display
        cv::erode(hsv_thresh, hsv_thresh, erodeElement,cv::Point(-1,-1),2);
    //    cv::imshow(WINDOW3, hsv_thresh);

        // Dilate then display
        cv::dilate(hsv_thresh, hsv_thresh, dilateElement);
        //cv::imshow(WINDOW4, hsv_thresh);

        // Blur image
        cv::GaussianBlur(hsv_thresh, hsv_thresh, cv::Size(9, 9), 1, 1);

        cv::Mat canny_output;
        std::vector<std::vector<cv::Point> > contours;
        cv::Rect rectangle(50, 50, 1, 1), rectangle_temp;
        std::vector<cv::Vec4i> hierarchy;

        /// Detect edges using canny
        int low_thresh = 100;
        cv::Canny( hsv_thresh, canny_output, low_thresh, low_thresh*2, 3 );
        /// Find contours
        cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /// Draw contours
        cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );


        for( int i = 0; i< contours.size(); i++ ) {
            cv::drawContours(drawing, contours, i, cv::Scalar( 0, 255, 0), 2, 8, hierarchy, 0, cv::Point() );
            rectangle_temp = cv::boundingRect(contours[i]);
            if (rectangle_temp.area() > rectangle.area()) {
                rectangle = rectangle_temp;
            }
         }

        cv::rectangle(drawing, rectangle, cv::Scalar( 255, 255, 0));
        if (display) {
            imshow(WINDOW1, drawing);
        }

        bucket.x = rectangle.x;
        bucket.y = rectangle.y;
        bucket_pixel_pub.publish(bucket);

        //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
        cv::waitKey(3);

        //Convert the CvImage to a ROS image message and publish it
        it_pub.publish(cv_ptr_raw->toImageMsg());
    }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "detect_bucket_forward");
	ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    if (display) {
        cv::namedWindow(WINDOW1, CV_WINDOW_AUTOSIZE); //another option is: CV_WINDOW_NORMAL
        cv::namedWindow(WINDOW2, CV_WINDOW_AUTOSIZE);
    }

    nh.getParam("/detect_bucket_forward/display", display);
    nh.getParam("/detect_bucket_forward/require_correct_state", require_correct_state);

    nh.getParam("/detect_bucket_forward/h_min", H_MIN);
    nh.getParam("/detect_bucket_forward/h_max", H_MAX);
    nh.getParam("/detect_bucket_forward/s_min", S_MIN);
    nh.getParam("/detect_bucket_forward/s_max", S_MAX);
    nh.getParam("/detect_bucket_forward/v_min", V_MIN);
    nh.getParam("/detect_bucket_forward/v_max", V_MAX);

    //image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/camera_forward/image_rect_color", 1, imageCallback);
    bucket_pixel_pub = nh.advertise<geometry_msgs::Point>("/detect_bucket_forward/bucket_pixel",1,true);

    if (display) {
        cv::namedWindow("trackbars",0);
        cv::createTrackbar("H_MIN", "trackbars", &H_MIN, H_TOP, on_trackbar);
        cv::createTrackbar("H_MAX", "trackbars", &H_MAX, H_TOP, on_trackbar);
        cv::createTrackbar("S_MIN", "trackbars", &S_MIN, S_TOP, on_trackbar);
        cv::createTrackbar("S_MAX", "trackbars", &S_MAX, S_TOP, on_trackbar);
        cv::createTrackbar("V_MIN", "trackbars", &V_MIN, V_TOP, on_trackbar);
        cv::createTrackbar("V_MAX", "trackbars", &V_MAX, V_TOP, on_trackbar);
    }

	ros::spin();

    if (display) {
        cv::destroyWindow(WINDOW1);
        cv::destroyWindow(WINDOW2);
    }
}
