#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

double ball_diameter = .0254; // in meters
double arm_diameter = .0508;
double arm1_length = .0965;
double arm2_length = .0635;
double arm3_length = .0508;
double claw_length = .0108;

int main(int argc, char **argv) {

	ros::init(argc, argv, "ball_marker");

	ros::NodeHandle nh;

	visualization_msgs::Marker ball_marker, arm1_marker, arm2_marker, arm3_marker, claw_marker;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
 	geometry_msgs::TransformStamped ball;
 	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_marker", 1000, true);
	ros::Rate rate(5.0);

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

	arm1_marker.header.frame_id = "arm1";
	arm1_marker.header.stamp = ros::Time();
	arm1_marker.id = 1;
	arm1_marker.type = visualization_msgs::Marker::CYLINDER;
	arm1_marker.action = visualization_msgs::Marker::ADD;
	arm1_marker.pose.position.x = 0;
	arm1_marker.pose.position.y = 0;
	arm1_marker.pose.position.z = 0;
	arm1_marker.pose.orientation.x = 0.0;
	arm1_marker.pose.orientation.y = 0.0;
	arm1_marker.pose.orientation.z = 0.0;
	arm1_marker.pose.orientation.w = 1.0;
	arm1_marker.scale.x = arm_diameter;
	arm1_marker.scale.y = arm_diameter;
	arm1_marker.scale.z = arm1_length;
	arm1_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm1_marker.color.r = 1.0;
	arm1_marker.color.g = 1.0;
	arm1_marker.color.b = 0.0;
	arm1_marker.lifetime = ros::Duration();

	arm2_marker.header.frame_id = "arm2";
	arm2_marker.header.stamp = ros::Time();
	arm2_marker.id = 2;
	arm2_marker.type = visualization_msgs::Marker::CYLINDER;
	arm2_marker.action = visualization_msgs::Marker::ADD;
	arm2_marker.pose.position.x = 0;
	arm2_marker.pose.position.y = 0;
	arm2_marker.pose.position.z = 0;
	arm2_marker.pose.orientation.x = 0.0;
	arm2_marker.pose.orientation.y = 0.0;
	arm2_marker.pose.orientation.z = 0.0;
	arm2_marker.pose.orientation.w = 1.0;
	arm2_marker.scale.x = arm_diameter;
	arm2_marker.scale.y = arm_diameter;
	arm2_marker.scale.z = arm1_length;
	arm2_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm2_marker.color.r = 1.0;
	arm2_marker.color.g = 1.0;
	arm2_marker.color.b = 0.0;
	arm2_marker.lifetime = ros::Duration();

	arm3_marker.header.frame_id = "arm3";
	arm3_marker.header.stamp = ros::Time();
	arm3_marker.id = 3;
	arm3_marker.type = visualization_msgs::Marker::CYLINDER;
	arm3_marker.action = visualization_msgs::Marker::ADD;
	arm3_marker.pose.position.x = 0;
	arm3_marker.pose.position.y = 0;
	arm3_marker.pose.position.z = 0;
	arm3_marker.pose.orientation.x = 0.0;
	arm3_marker.pose.orientation.y = 0.0;
	arm3_marker.pose.orientation.z = 0.0;
	arm3_marker.pose.orientation.w = 1.0;
	arm3_marker.scale.x = arm_diameter;
	arm3_marker.scale.y = arm_diameter;
	arm3_marker.scale.z = arm1_length;
	arm3_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm3_marker.color.r = 1.0;
	arm3_marker.color.g = 1.0;
	arm3_marker.color.b = 0.0;
	arm3_marker.lifetime = ros::Duration();

	while(ros::ok()) {
		try {
        	ball = tf_buffer.lookupTransform("left_camera", "ball", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm_base", "arm1", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm1", "arm2", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm2", "arm3", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm3", "claw", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	//marker.pose.position.x = ball.transform.translation.x;
		//marker.pose.position.y = ball.transform.translation.y;

		marker_pub.publish(ball_marker);
		marker_pub.publish(arm1_marker);
		marker_pub.publish(arm2_marker);
		marker_pub.publish(arm3_marker);
		marker_pub.publish(claw_marker);

    	rate.sleep();
	}

}