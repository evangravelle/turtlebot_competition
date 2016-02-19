#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher location_pub;
geometry_msgs::Point left_image_pixel, right_image_pixel;
geometry_msgs::TransformStamped ball;

int image_width, image_height;
double left_theta, right_theta, dist;
double ball_diameter = .033; // in meters
double baseline_length = .248; // in meters
double camera_height = .406; // in meters
double camera_from_center = .038; // in y direction, in meters

// Assumes a horizontal camera view of 90 degrees


void leftLocationCallback(const geometry_msgs::Point::ConstPtr &pointPtr) {

	static tf2_ros::TransformBroadcaster tf_br;
	ball.header.stamp = ros::Time::now();

	left_image_pixel.x = pointPtr->x;

	//std::cout << (left_image_pixel.x - image_width/2.0)/(image_width/2.0) << std::endl;

	left_theta = atan((left_image_pixel.x - image_width/2.0)/(image_width/2.0));

	//std::cout << left_theta << std::endl;

	dist = 10;

	ball.transform.translation.x = dist*cos(left_theta);
	ball.transform.translation.y = dist*sin(left_theta);

	tf_br.sendTransform(ball);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "locate_ball"); // Ros Initialize

	ros::NodeHandle nh;

	ros::Subscriber left_image_sub = nh.subscribe<geometry_msgs::Point>("/left_image_ball_pixel", 1, leftLocationCallback);
	// ros::Subscriber right_image_sub = nh.subscribe<geometry_msgs::Point>("/right_image_ball_pixel", 1, rightLocationCallback);
	// location_pub = nh.advertise<geometry_msgs::Pose>("/ball_location", 1, true);

	// Defaults
	image_width = 640;
	image_height = 480;

	nh.getParam("/usb_cam/image_width", image_width);
	nh.getParam("/usb_cam/image_height", image_height);
	nh.setParam("/baseline_length", baseline_length); 

	left_image_pixel.x = image_width/2.0;
	left_image_pixel.y = image_height/2.0;

	ball.header.frame_id = "left_camera";
	ball.child_frame_id = "ball";
	ball.transform.translation.x = 0.0;
	ball.transform.translation.y = 0.0;
	ball.transform.translation.z = -camera_height + ball_diameter/2.0;
	ball.transform.rotation.x = 0.0;
	ball.transform.rotation.y = 0.0;
	ball.transform.rotation.z = 0.0;
	ball.transform.rotation.w = 1.0;

	ros::spin();

}
