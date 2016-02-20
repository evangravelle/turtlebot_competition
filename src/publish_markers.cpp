#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

double ball_diameter = .0254; // in meters
double arm_diameter = .03;
double arm_1_length = .102;
double arm_2_length = .076;
double claw_length = .15;
double camera_thickness = .04;
double camera_width = .07;
double camera_height = .05;

int main(int argc, char **argv) {

	ros::init(argc, argv, "publish_markers");

	ros::NodeHandle nh;

	visualization_msgs::Marker ball_marker, arm_1_marker, arm_2_marker, claw_marker, camera_forward_marker, camera_up_marker, camera_down_marker;
	//geometry_msgs::TransformStamped ball, arm_1, arm_2, claw;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
 	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_marker", 1000, true);
 	ros::Publisher arm_1_pub = nh.advertise<visualization_msgs::Marker>("/arm_1_marker", 1000, true);
 	ros::Publisher arm_2_pub = nh.advertise<visualization_msgs::Marker>("/arm_2_marker", 1000, true);
 	ros::Publisher claw_pub = nh.advertise<visualization_msgs::Marker>("/claw_marker", 1000, true);
 	ros::Publisher camera_forward_pub = nh.advertise<visualization_msgs::Marker>("/camera_forward_marker", 1000, true);
 	ros::Publisher camera_up_pub = nh.advertise<visualization_msgs::Marker>("/camera_up_marker", 1000, true);
  	ros::Publisher camera_down_pub = nh.advertise<visualization_msgs::Marker>("/camera_down_marker", 1000, true);
	ros::Rate rate(10.0);

	camera_forward_marker.header.frame_id = "camera_forward";
	camera_forward_marker.header.stamp = ros::Time();
	camera_forward_marker.id = 3;
	camera_forward_marker.type = visualization_msgs::Marker::CUBE;
	camera_forward_marker.action = visualization_msgs::Marker::ADD;
	camera_forward_marker.pose.position.x = 0;
	camera_forward_marker.pose.position.y = 0;
	camera_forward_marker.pose.position.z = 0;
	camera_forward_marker.pose.orientation.x = 0.0;
	camera_forward_marker.pose.orientation.y = 0.0;
	camera_forward_marker.pose.orientation.z = 0.0;
	camera_forward_marker.pose.orientation.w = 1.0;
	camera_forward_marker.scale.x = camera_thickness;
	camera_forward_marker.scale.y = camera_width;
	camera_forward_marker.scale.z = camera_height;
	camera_forward_marker.color.a = 1.0; // Don't forget to set the alpha!
	camera_forward_marker.color.r = 0.0;
	camera_forward_marker.color.g = 0.0;
	camera_forward_marker.color.b = 0.0;
	camera_forward_marker.lifetime = ros::Duration();

	camera_up_marker.header.frame_id = "camera_up";
	camera_up_marker.header.stamp = ros::Time();
	camera_up_marker.id = 3;
	camera_up_marker.type = visualization_msgs::Marker::CUBE;
	camera_up_marker.action = visualization_msgs::Marker::ADD;
	camera_up_marker.pose.position.x = 0;
	camera_up_marker.pose.position.y = 0;
	camera_up_marker.pose.position.z = 0;
	camera_up_marker.pose.orientation.x = 0.0;
	camera_up_marker.pose.orientation.y = 0.0;
	camera_up_marker.pose.orientation.z = 0.0;
	camera_up_marker.pose.orientation.w = 1.0;
	camera_up_marker.scale.x = camera_thickness;
	camera_up_marker.scale.y = camera_width;
	camera_up_marker.scale.z = camera_height;
	camera_up_marker.color.a = 1.0; // Don't forget to set the alpha!
	camera_up_marker.color.r = 0.0;
	camera_up_marker.color.g = 0.0;
	camera_up_marker.color.b = 0.0;
	camera_up_marker.lifetime = ros::Duration();

	camera_down_marker.header.frame_id = "camera_down";
	camera_down_marker.header.stamp = ros::Time();
	camera_down_marker.id = 3;
	camera_down_marker.type = visualization_msgs::Marker::CUBE;
	camera_down_marker.action = visualization_msgs::Marker::ADD;
	camera_down_marker.pose.position.x = 0;
	camera_down_marker.pose.position.y = 0;
	camera_down_marker.pose.position.z = 0;
	camera_down_marker.pose.orientation.x = 0.0;
	camera_down_marker.pose.orientation.y = 0.0;
	camera_down_marker.pose.orientation.z = 0.0;
	camera_down_marker.pose.orientation.w = 1.0;
	camera_down_marker.scale.x = camera_thickness;
	camera_down_marker.scale.y = camera_width;
	camera_down_marker.scale.z = camera_height;
	camera_down_marker.color.a = 1.0; // Don't forget to set the alpha!
	camera_down_marker.color.r = 0.0;
	camera_down_marker.color.g = 0.0;
	camera_down_marker.color.b = 0.0;
	camera_down_marker.lifetime = ros::Duration();

	ball_marker.header.frame_id = "ball";
	ball_marker.header.stamp = ros::Time();
	ball_marker.id = 0;
	ball_marker.type = visualization_msgs::Marker::SPHERE;
	ball_marker.action = visualization_msgs::Marker::ADD;
	ball_marker.pose.position.x = 0;
	ball_marker.pose.position.y = 0;
	ball_marker.pose.position.z = 0;
	ball_marker.pose.orientation.x = 0.0;
	ball_marker.pose.orientation.y = 0.0;
	ball_marker.pose.orientation.z = 0.0;
	ball_marker.pose.orientation.w = 1.0;
	ball_marker.scale.x = ball_diameter;
	ball_marker.scale.y = ball_diameter;
	ball_marker.scale.z = ball_diameter;
	ball_marker.color.a = 1.0; // Don't forget to set the alpha!
	ball_marker.color.r = 1.0;
	ball_marker.color.g = 0.6;
	ball_marker.color.b = 0.0;
	ball_marker.lifetime = ros::Duration();

	arm_1_marker.header.frame_id = "arm_1";
	arm_1_marker.header.stamp = ros::Time();
	arm_1_marker.id = 1;
	arm_1_marker.type = visualization_msgs::Marker::CYLINDER;
	arm_1_marker.action = visualization_msgs::Marker::ADD;
	arm_1_marker.pose.position.x = 0;
	arm_1_marker.pose.position.y = 0;
	arm_1_marker.pose.position.z = arm_1_length/2;
	arm_1_marker.pose.orientation.x = 0.0;
	arm_1_marker.pose.orientation.y = 0.0;
	arm_1_marker.pose.orientation.z = 0.0;
	arm_1_marker.pose.orientation.w = 1.0;
	arm_1_marker.scale.x = arm_diameter;
	arm_1_marker.scale.y = arm_diameter;
	arm_1_marker.scale.z = arm_1_length;
	arm_1_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm_1_marker.color.r = 1.0;
	arm_1_marker.color.g = 1.0;
	arm_1_marker.color.b = 0.0;
	arm_1_marker.lifetime = ros::Duration();

	arm_2_marker.header.frame_id = "arm_2";
	arm_2_marker.header.stamp = ros::Time();
	arm_2_marker.id = 2;
	arm_2_marker.type = visualization_msgs::Marker::CYLINDER;
	arm_2_marker.action = visualization_msgs::Marker::ADD;
	arm_2_marker.pose.position.x = 0;
	arm_2_marker.pose.position.y = 0;
	arm_2_marker.pose.position.z = arm_2_length/2;
	arm_2_marker.pose.orientation.x = 0.0;
	arm_2_marker.pose.orientation.y = 0.0;
	arm_2_marker.pose.orientation.z = 0.0;
	arm_2_marker.pose.orientation.w = 1.0;
	arm_2_marker.scale.x = arm_diameter;
	arm_2_marker.scale.y = arm_diameter;
	arm_2_marker.scale.z = arm_2_length;
	arm_2_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm_2_marker.color.r = 1.0;
	arm_2_marker.color.g = 1.0;
	arm_2_marker.color.b = 0.0;
	arm_2_marker.lifetime = ros::Duration();

	claw_marker.header.frame_id = "claw";
	claw_marker.header.stamp = ros::Time();
	claw_marker.id = 3;
	claw_marker.type = visualization_msgs::Marker::CYLINDER;
	claw_marker.action = visualization_msgs::Marker::ADD;
	claw_marker.pose.position.x = 0;
	claw_marker.pose.position.y = 0;
	claw_marker.pose.position.z = claw_length/2;
	claw_marker.pose.orientation.x = 0.0;
	claw_marker.pose.orientation.y = 0.0;
	claw_marker.pose.orientation.z = 0.0;
	claw_marker.pose.orientation.w = 1.0;
	claw_marker.scale.x = arm_diameter;
	claw_marker.scale.y = arm_diameter;
	claw_marker.scale.z = claw_length;
	claw_marker.color.a = 1.0; // Don't forget to set the alpha!
	claw_marker.color.r = 0.0;
	claw_marker.color.g = 0.5;
	claw_marker.color.b = 0.0;
	claw_marker.lifetime = ros::Duration();

	while(ros::ok()) {

		// try {
        	// ball = tf_buffer.lookupTransform("camera_forward", "ball", ros::Time(0));
        	//arm_1 = tf_buffer.lookupTransform("arm_1", "arm_2", ros::Time(0));
        	//arm_2 = tf_buffer.lookupTransform("arm_2", "arm_claw", ros::Time(0));
    	// }
		// catch (tf2::TransformException &ex) {
        //	 ROS_WARN("%s",ex.what());
        //	 ros::Duration(0.02).sleep();
        //	 continue;
    	// }

		marker_pub.publish(ball_marker);
		arm_1_pub.publish(arm_1_marker);
		arm_2_pub.publish(arm_2_marker);
		claw_pub.publish(claw_marker);
		camera_forward_pub.publish(camera_forward_marker);
		camera_up_pub.publish(camera_up_marker);
		camera_down_pub.publish(camera_down_marker);

    	rate.sleep();
	}

}