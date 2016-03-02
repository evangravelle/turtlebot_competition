#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

ros::Publisher location_pub;
geometry_msgs::Point forward_image_pixel;
geometry_msgs::TransformStamped bucket;

double inches_to_meters = 0.0254;
double ball_diameter = 1.125*inches_to_meters; // in meters
double baseline_length = 10.5*inches_to_meters;
double camera_forward_dist_from_ground = 16.0*inches_to_meters;
int image_width, image_height;
double xy_angle, dist;
double bucket_dist_away = 1.0;

// Assumes a horizontal camera view of 90 degrees


void forwardLocationCallback(const geometry_msgs::Point::ConstPtr &pointPtr) {

	static tf2_ros::TransformBroadcaster tf_br;
	bucket.header.stamp = ros::Time::now();

	forward_image_pixel.x = pointPtr->x;
	forward_image_pixel.y = pointPtr->y;

	bucket.transform.translation.x = bucket_dist_away;
	bucket.transform.translation.y = bucket_dist_away - 2*bucket_dist_away*forward_image_pixel.x/(image_width - 1);
	bucket.transform.translation.z = 0;

	tf_br.sendTransform(bucket);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "locate_bucket_forward"); // Ros Initialize

	ros::NodeHandle nh;

	ros::Subscriber image_sub = nh.subscribe<geometry_msgs::Point>("/detect_bucket_forward/bucket_pixel", 1, forwardLocationCallback);
	// ros::Subscriber right_image_sub = nh.subscribe<geometry_msgs::Point>("/right_image_ball_pixel", 1, rightLocationCallback);
	// location_pub = nh.advertise<geometry_msgs::Pose>("/ball_location", 1, true);

	// Defaults
	image_width = 640;
	image_height = 480;

	nh.getParam("/camera_forward/image_width", image_width);
	nh.getParam("/camera_forward/image_height", image_height);
	nh.setParam("/baseline_length", baseline_length); 

	forward_image_pixel.x = image_width/2.0;
	forward_image_pixel.y = image_height/2.0;

	bucket.header.frame_id = "camera_forward";
	bucket.child_frame_id = "bucket_forward";
	bucket.transform.translation.x = 0.0;
	bucket.transform.translation.y = 0.0;
	bucket.transform.translation.z = 0.0;
	bucket.transform.rotation.x = 0.0;
	bucket.transform.rotation.y = 0.0;
	bucket.transform.rotation.z = 0.0;
	bucket.transform.rotation.w = 1.0;

	ros::spin();

}
