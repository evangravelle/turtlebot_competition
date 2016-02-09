#include <ros/ros.h>
#include <tf2/transform_broadcaster.h>

double baseline_length = 9.75; // in inches
double camera_height = 16.0; // in inches
double camera_from_center = 1.5; // in y direction, in inches

int main(int argc, char **argv) {

	ros::init(argc, argv, "turtlebot_tf");
	ros::NodeHandle nh;
   
	tf::TransformBroadcaster tf_br;
	tf::Transform left_camera, right_camera;

	
	ros::Rate rate(2.0);
	while (node.ok()){
		left_camera.header.stamp = ros::Time::now();
		left_camera.header.frame_id = "turtlebot"
		left_camera.child_frame_id = "left_camera";
		left_camera.setOrigin( tf::Vector3(-baseline_length/2.0, camera_from_center, camera_height) );
		left_camera.setRotation( tf::Quaternion(0, 0, 0, 1) );

		right_camera.header.stamp = left_camera.header.stamp;
		right_camera.header.frame_id = "turtlebot"
		right_camera.child_frame_id = "right_camera";
		right_camera.setOrigin( tf::Vector3(baseline_length/2.0, camera_from_center, camera_height) );
		right_camera.setRotation( tf::Quaternion(0, 0, 0, 1) );

		tf_br.sendTransform(left_camera);
		tf_br.sendTransform(right_camera);

		rate.sleep();
	}

}