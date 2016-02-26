#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

ros::Publisher location_pub;
geometry_msgs::Point forward_image_pixel;
geometry_msgs::TransformStamped ball;

double inches_to_meters = 0.0254;
double ball_diameter = 1.125*inches_to_meters; // in meters
double baseline_length = 10.5*inches_to_meters;
double camera_forward_dist_from_ground = 16.0*inches_to_meters;
int image_width, image_height;
double xy_angle, dist;

// Assumes a horizontal camera view of 90 degrees


void downLocationCallback(const geometry_msgs::Point::ConstPtr &pointPtr) {

	static tf2_ros::TransformBroadcaster tf_br;
	ball.header.stamp = ros::Time::now();

	forward_image_pixel.x = pointPtr->x;
	forward_image_pixel.y = pointPtr->y;

	ball.transform.translation.x = exp(8.1*pow(10,-6)*pow(forward_image_pixel.y,2) - 0.0091*forward_image_pixel.y + 5.1283)*inches_to_meters;
	ball.transform.translation.y = ball.transform.translation.y * (forward_image_pixel.x - image_width/2.0)/(image_width/2.0);

	tf_br.sendTransform(ball);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "locate_ball_down"); // Ros Initialize

	ros::NodeHandle nh;

	ros::Subscriber image_sub = nh.subscribe<geometry_msgs::Point>("/detect_ball_down/ball_pixel", 1, downLocationCallback);
	// location_pub = nh.advertise<geometry_msgs::Pose>("/ball_location", 1, true);

	// Defaults
	image_width = 640;
	image_height = 480;

	nh.getParam("/camera_down/image_width", image_width);
	nh.getParam("/camera_down/image_height", image_height);

	forward_image_pixel.x = image_width/2.0;
	forward_image_pixel.y = image_height/2.0;

	ball.header.frame_id = "camera_down";
	ball.child_frame_id = "ball_down";
	ball.transform.translation.x = 0.0;
	ball.transform.translation.y = 0.0;
	ball.transform.translation.z = -camera_forward_dist_from_ground + ball_diameter/2.0;
	ball.transform.rotation.x = 0.0;
	ball.transform.rotation.y = 0.0;
	ball.transform.rotation.z = 0.0;
	ball.transform.rotation.w = 1.0;

	ros::spin();

}
