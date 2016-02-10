#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

double ball_diameter = .0254; // in meters

int main(int argc, char **argv) {

	ros::init(argc, argv, "ball_marker");

	ros::NodeHandle nh;

	visualization_msgs::Marker marker;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
 	geometry_msgs::TransformStamped ball;
 	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/ball_marker", 1000, true);
	ros::Rate rate(10.0);

	marker.header.frame_id = "ball";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = ball_diameter;
	marker.scale.y = ball_diameter;
	marker.scale.z = ball_diameter;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.6;
	marker.color.b = 0.0;

	while(ros::ok()) {
		try {
        	ball = tf_buffer.lookupTransform("left_camera", "ball", ros::Time(0));
    	}
		catch (tf2::TransformException &ex) {
        	ROS_WARN("%s",ex.what());
        	ros::Duration(1.0).sleep();
        	continue;
    	}

    	//marker.pose.position.x = ball.transform.translation.x;
		//marker.pose.position.y = ball.transform.translation.y;

		marker_pub.publish(marker);

    	rate.sleep();
	}

}