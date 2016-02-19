#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

double ball_diameter = .0254; // in meters
double arm_diameter = .0508;
double arm_1_length = .0965;
double arm_2_length = .0635;
double arm_3_length = .0508;
double claw_length = .0108;

int main(int argc, char **argv) {

	ros::init(argc, argv, "publish_markers");

	ros::NodeHandle nh;

	visualization_msgs::Marker ball_marker, arm_1_marker, arm_2_marker, arm_3_marker, claw_marker;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
 	geometry_msgs::TransformStamped ball;
 	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_marker", 10, true);
 	ros::Publisher arm_1_pub = nh.advertise<visualization_msgs::Marker>("/arm_1_marker", 10, true);
 	ros::Publisher arm_2_pub = nh.advertise<visualization_msgs::Marker>("/arm_2_marker", 10, true);
 	ros::Publisher arm_3_pub = nh.advertise<visualization_msgs::Marker>("/arm_3_marker", 10, true);
 	ros::Publisher claw_pub = nh.advertise<visualization_msgs::Marker>("/claw_marker", 10, true);
	ros::Rate rate(10.0);

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
	arm_1_marker.pose.position.z = 0;
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
	arm_2_marker.pose.position.z = 0;
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

	arm_3_marker.header.frame_id = "arm_3";
	arm_3_marker.header.stamp = ros::Time();
	arm_3_marker.id = 3;
	arm_3_marker.type = visualization_msgs::Marker::CYLINDER;
	arm_3_marker.action = visualization_msgs::Marker::ADD;
	arm_3_marker.pose.position.x = 0;
	arm_3_marker.pose.position.y = 0;
	arm_3_marker.pose.position.z = 0;
	arm_3_marker.pose.orientation.x = 0.0;
	arm_3_marker.pose.orientation.y = 0.0;
	arm_3_marker.pose.orientation.z = 0.0;
	arm_3_marker.pose.orientation.w = 1.0;
	arm_3_marker.scale.x = arm_diameter;
	arm_3_marker.scale.y = arm_diameter;
	arm_3_marker.scale.z = arm_3_length;
	arm_3_marker.color.a = 1.0; // Don't forget to set the alpha!
	arm_3_marker.color.r = 1.0;
	arm_3_marker.color.g = 1.0;
	arm_3_marker.color.b = 0.0;
	arm_3_marker.lifetime = ros::Duration();

	claw_marker.header.frame_id = "claw";
	claw_marker.header.stamp = ros::Time();
	claw_marker.id = 3;
	claw_marker.type = visualization_msgs::Marker::CYLINDER;
	claw_marker.action = visualization_msgs::Marker::ADD;
	claw_marker.pose.position.x = 0;
	claw_marker.pose.position.y = 0;
	claw_marker.pose.position.z = 0;
	claw_marker.pose.orientation.x = 0.0;
	claw_marker.pose.orientation.y = 0.0;
	claw_marker.pose.orientation.z = 0.0;
	claw_marker.pose.orientation.w = 1.0;
	claw_marker.scale.x = arm_diameter;
	claw_marker.scale.y = arm_diameter;
	claw_marker.scale.z = claw_length;
	claw_marker.color.a = 1.0; // Don't forget to set the alpha!
	claw_marker.color.r = 1.0;
	claw_marker.color.g = 1.0;
	claw_marker.color.b = 0.0;
	claw_marker.lifetime = ros::Duration();

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
        	ball = tf_buffer.lookupTransform("arm_base", "arm_1", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm_1", "arm_2", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm_2", "arm_3", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	try {
        	ball = tf_buffer.lookupTransform("arm_3", "claw", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	//marker.pose.position.x = ball.transform.translation.x;
		//marker.pose.position.y = ball.transform.translation.y;

		marker_pub.publish(ball_marker);
		arm_1_pub.publish(arm_1_marker);
		arm_2_pub.publish(arm_2_marker);
		arm_3_pub.publish(arm_3_marker);
		claw_pub.publish(claw_marker);

    	rate.sleep();
	}

}