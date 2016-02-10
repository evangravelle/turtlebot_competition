#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

double baseline_length = .248; // in inches
double camera_height = .406; // in inches
double camera_from_center = .038; // in y direction, in inches

int main(int argc, char **argv) {

	ros::init(argc, argv, "turtlebot_tf");
	ros::NodeHandle nh;
   
	tf2_ros::TransformBroadcaster tf_br;
	geometry_msgs::TransformStamped left_camera, right_camera;

	nh.setParam("/baseline_length", baseline_length); 
	nh.setParam("/camera_height", camera_height);
	nh.setParam("/camera_from_center", camera_from_center);

	ros::Rate rate(20.0);
	while (ros::ok()){
		left_camera.header.stamp = ros::Time::now();
		left_camera.header.frame_id = "base_footprint";
		left_camera.child_frame_id = "left_camera";
		left_camera.transform.translation.x = camera_from_center;
		left_camera.transform.translation.y = baseline_length/2.0;
		left_camera.transform.translation.z = camera_height;
		left_camera.transform.rotation.x = 0;
		left_camera.transform.rotation.y = 0;
		left_camera.transform.rotation.z = 0;
		left_camera.transform.rotation.w = 1;

		right_camera.header.stamp = left_camera.header.stamp;
		right_camera.header.frame_id = "base_footprint";
		right_camera.child_frame_id = "right_camera";
		right_camera.transform.translation.x = camera_from_center;
		right_camera.transform.translation.y = -baseline_length/2.0;
		right_camera.transform.translation.z = camera_height;
		right_camera.transform.rotation.x = 0;
		right_camera.transform.rotation.y = 0;
		right_camera.transform.rotation.z = 0;
		right_camera.transform.rotation.w = 1;

		tf_br.sendTransform(left_camera);
		tf_br.sendTransform(right_camera);

		rate.sleep();
	}

}