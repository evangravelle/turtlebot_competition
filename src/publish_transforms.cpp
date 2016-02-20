#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <coconuts_common/ArmStatus.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

double baseline_length = .248; // in inches
double camera_height = .406; // in inches
double camera_from_center = .038; // in y direction, in inches
double arm_base_forward = .197;
double arm_base_up = .178;
double arm_diameter = .03;
double arm_1_length = .102;
double arm_2_length = .076;
double claw_length = .15;

geometry_msgs::TransformStamped camera_forward, camera_up, camera_down, arm_base, arm_1, arm_2, claw;

void armStatusCallback(const coconuts_common::ArmStatus::ConstPtr &arm_status) {

	//static tf2_ros::TransformBroadcaster tf_br_static;

	arm_base.transform.rotation.y = arm_status->motor_positions[0].position;
	arm_1.transform.rotation.y = arm_status->motor_positions[1].position;;
	arm_2.transform.rotation.y = arm_status->motor_positions[2].position;;
	claw.transform.rotation.y = arm_status->motor_positions[3].position;;

	//tf_br_static.sendTransform(arm_1);
	//tf_br_static.sendTransform(arm_2);
	//tf_br_static.sendTransform(claw);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "publish_transforms");
	ros::NodeHandle nh;

	ros::Subscriber arm_sub = nh.subscribe<coconuts_common::ArmStatus>("/arm_status", 1, armStatusCallback);
	tf2_ros::TransformBroadcaster tf_br;

	nh.setParam("/baseline_length", baseline_length); 
	nh.setParam("/camera_height", camera_height);
	nh.setParam("/camera_from_center", camera_from_center);

	ros::Rate rate(5.0);
	
	// Initialize transforms
	camera_forward.header.stamp = ros::Time::now();
	camera_forward.header.frame_id = "base_footprint";
	camera_forward.child_frame_id = "camera_forward";
	camera_forward.transform.translation.x = -camera_from_center;
	camera_forward.transform.translation.y = baseline_length/2.0;
	camera_forward.transform.translation.z = camera_height;
	camera_forward.transform.rotation.x = 0;
	camera_forward.transform.rotation.y = 0;
	camera_forward.transform.rotation.z = 0;
	camera_forward.transform.rotation.w = 1;

	tf2::Quaternion camera_up_quat;
	camera_up_quat.setRPY(0, -M_PI/2, 0);
	camera_up.header.stamp = ros::Time::now();
	camera_up.header.frame_id = "base_footprint";
	camera_up.child_frame_id = "camera_up";
	camera_up.transform.translation.x = -camera_from_center;
	camera_up.transform.translation.y = -baseline_length/2.0;
	camera_up.transform.translation.z = camera_height;
	camera_up.transform.rotation.x = camera_up_quat.x();
	camera_up.transform.rotation.y = camera_up_quat.y();
	camera_up.transform.rotation.z = camera_up_quat.z();
	camera_up.transform.rotation.w = camera_up_quat.w();

	tf2::Quaternion camera_down_quat;
	camera_down_quat.setRPY(0, -M_PI/2.0, 0);
	camera_down.header.stamp = ros::Time::now();
	camera_down.header.frame_id = "arm_2";
	camera_down.child_frame_id = "camera_down";
	camera_down.transform.translation.x = -arm_diameter;
	camera_down.transform.translation.y = 0;
	camera_down.transform.translation.z = arm_2_length/2.0;
	camera_down.transform.rotation.x = camera_down_quat.x();
	camera_down.transform.rotation.y = camera_down_quat.y();
	camera_down.transform.rotation.z = camera_down_quat.z();
	camera_down.transform.rotation.w = camera_down_quat.w();

	arm_base.header.stamp = ros::Time::now();
	arm_base.header.frame_id = "base_footprint";
	arm_base.child_frame_id = "arm_base";
	arm_base.transform.translation.x = arm_base_forward;
	arm_base.transform.translation.y = 0;
	arm_base.transform.translation.z = arm_base_up;
	arm_base.transform.rotation.x = 0;
	arm_base.transform.rotation.y = 0;
	arm_base.transform.rotation.z = 0;
	arm_base.transform.rotation.w = 1;

	tf2::Quaternion arm_1_quat;
	arm_1_quat.setRPY(0, M_PI/8.0, 0);
	arm_1.header.stamp = ros::Time::now();
	arm_1.header.frame_id = "arm_base";
	arm_1.child_frame_id = "arm_1";
	arm_1.transform.translation.x = 0;
	arm_1.transform.translation.y = 0;
	arm_1.transform.translation.z = 0;
	arm_1.transform.rotation.x = arm_1_quat.x();
	arm_1.transform.rotation.y = arm_1_quat.y();
	arm_1.transform.rotation.z = arm_1_quat.z();
	arm_1.transform.rotation.w = arm_1_quat.w();

	tf2::Quaternion arm_2_quat;
	arm_2_quat.setRPY(0, M_PI/3.0, 0);
	arm_2.header.stamp = ros::Time::now();
	arm_2.header.frame_id = "arm_1";
	arm_2.child_frame_id = "arm_2";
	arm_2.transform.translation.x = 0;
	arm_2.transform.translation.y = 0;
	arm_2.transform.translation.z = arm_1_length;
	arm_2.transform.rotation.x = arm_2_quat.x();
	arm_2.transform.rotation.y = arm_2_quat.y();
	arm_2.transform.rotation.z = arm_2_quat.z();
	arm_2.transform.rotation.w = arm_2_quat.w();

	tf2::Quaternion claw_quat;
	claw_quat.setRPY(0, M_PI/16.0, 0);
	claw.header.stamp = ros::Time::now();
	claw.header.frame_id = "arm_2";
	claw.child_frame_id = "claw";
	claw.transform.translation.x = 0;
	claw.transform.translation.y = 0;
	claw.transform.translation.z = arm_2_length;
	claw.transform.rotation.x = claw_quat.x();
	claw.transform.rotation.y = claw_quat.y();
	claw.transform.rotation.z = claw_quat.z();
	claw.transform.rotation.w = claw_quat.w();


	while (ros::ok()){

		camera_forward.header.stamp = ros::Time::now();
		camera_up.header.stamp = ros::Time::now();
		camera_down.header.stamp = ros::Time::now();
		arm_base.header.stamp = ros::Time::now();
		arm_1.header.stamp = ros::Time::now();
		arm_2.header.stamp = ros::Time::now();
		claw.header.stamp = ros::Time::now();

		tf_br.sendTransform(camera_forward);
		tf_br.sendTransform(camera_up);
		tf_br.sendTransform(camera_down);
		tf_br.sendTransform(arm_base);
		tf_br.sendTransform(arm_1);
		tf_br.sendTransform(arm_2);
		tf_br.sendTransform(claw);

		rate.sleep();
	}

}