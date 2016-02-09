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
double ball_diameter = 1.3; // in inches

// Assumes a horizontal camera view of 90 degrees


void leftLocationCallback(const geometry_msgs::Point::ConstPtr& pointPtr) {

	static tf2_ros::TransformBroadcaster tf_br;
	ball.header.stamp = ross::Time::now();


	left_image_pixel.x = pointPtr->x;
	left_image_pixel.y = pointPtr->y;

	left_theta = atan((left_image_pixel.x - image_width/2.0)/(image_width/2.0));

	dist = baseline_length * sin(M_PI/2 - right_theta) / sin(left_theta + right_theta);

	ball.transform.position.x = dist*sin(left_theta);
	ball.transform.position.y = dist*sin(right_theta);

	tf_br.sendTransform(ball);

}

void rightLocationCallback(const geometry_msgs::Point::ConstPtr& pointPtr) {

	right_image_pixel.x = pointPtr->x;
	right_image_pixel.y = pointPtr->y;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "locate_ball"); // Ros Initialize

	ros::NodeHandle nh;

	ros::Subscriber left_image_sub = nh.subscribe<geometry_msgs::Point>("/left_image_ball_pixel", 1, leftLocationCallback);
	ros::Subscriber right_image_sub = nh.subscribe<geometry_msgs::Point>("/right_image_ball_pixel", 1, rightLocationCallback);
	location_pub = nh.advertise<geometry_msgs::Pose>("/ball_location", 1, true);

	nh.getParam("/usb_cam/image_width", image_width);
	nh.getParam("/usb_cam/image_height", image_height);
	nh.setParam("/baseline_length", baseline_length); 
	nh.setParam("/camera_height", camera_height);
	nh.setParam("/camera_from_center", camera_from_center);

	right_image_pixel.x = image_width/2.0;
	left_image_pixel.y = image_height/2.0;

	ball.header.frame_id = "left_camera";
	ball.child_frame_id = "ball";
	ball.transform.position.x = 0.0;
	ball.transform.position.y = 0.0;
	ball.transform.position.z = -camera_height + ball_diameter/2.0;
	ball.transform.orientation.x = 0.0;
	ball.transform.orientation.y = 0.0;
	ball.transform.orientation.z = 0.0;
	ball.transform.orientation.w = 1.0;

	ros::spin();

}