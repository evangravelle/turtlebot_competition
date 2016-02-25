#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <coconuts_common/ArmStatus.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

double inches_to_meters = 0.0254;
double baseline_length = 10.5*inches_to_meters;
double camera_forward_dist_from_ground = 16.0*inches_to_meters;
double camera_up_dist_from_ground = camera_forward_dist_from_ground + 2.5*inches_to_meters;
double camera_forward_from_center = 1.5*inches_to_meters; // in x direction
double camera_up_from_center = 2.5*inches_to_meters + camera_forward_from_center; // in x direction
double arm_base_forward = 7.0*inches_to_meters;
double arm_base_up = 7.0*inches_to_meters;
double arm_diameter = 1.5*inches_to_meters;
double arm_1_length = 3.75*inches_to_meters;
double arm_2_length = 4.75*inches_to_meters;
double claw_length = 3.75*inches_to_meters;
double arm_1_pitch, arm_2_pitch, claw_pitch;

geometry_msgs::TransformStamped camera_forward, camera_up, camera_down, arm_base, arm_1, arm_2, claw;
tf2::Quaternion arm_1_quat, arm_2_quat, claw_quat;

void armStatusCallback(const coconuts_common::ArmStatus arm_status) {

	static tf2_ros::TransformBroadcaster tf_br_static;

	arm_1.header.stamp = ros::Time::now();
	arm_2.header.stamp = ros::Time::now();
	claw.header.stamp = ros::Time::now();

	arm_1_pitch = (M_PI/180.0) * (515.0 - arm_status.motor_positions[0].position)/5.0;
	arm_2_pitch = (M_PI/180.0) * (794.0 - arm_status.motor_positions[1].position)/5.0;
	claw_pitch = (M_PI/180.0) * (472.0 - arm_status.motor_positions[2].position)/5.0;

	arm_1_quat.setRPY(0, arm_1_pitch, 0);
	arm_2_quat.setRPY(0, arm_2_pitch, 0);
	claw_quat.setRPY(0, claw_pitch, 0);

	arm_1.transform.rotation.x = arm_1_quat.x();
	arm_1.transform.rotation.y = arm_1_quat.y();
	arm_1.transform.rotation.z = arm_1_quat.z();
	arm_1.transform.rotation.w = arm_1_quat.w();

	arm_2.transform.rotation.x = arm_2_quat.x();
	arm_2.transform.rotation.y = arm_2_quat.y();
	arm_2.transform.rotation.z = arm_2_quat.z();
	arm_2.transform.rotation.w = arm_2_quat.w();

	claw.transform.rotation.x = claw_quat.x();
	claw.transform.rotation.y = claw_quat.y();
	claw.transform.rotation.z = claw_quat.z();
	claw.transform.rotation.w = claw_quat.w();

	tf_br_static.sendTransform(arm_1);
	tf_br_static.sendTransform(arm_2);
	tf_br_static.sendTransform(claw);

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "publish_transforms");
	ros::NodeHandle nh;

	tf2_ros::TransformBroadcaster tf_br;
	ros::Subscriber arm_sub = nh.subscribe<coconuts_common::ArmStatus>("/arm_status", 1000, armStatusCallback);

	//nh.setParam("/baseline_length", baseline_length); 
	//nh.setParam("/camera_dist_from_ground", camera_dist_from_ground);
	//nh.setParam("/camera_forward_from_center", camera_forward_from_center);
	//nh.setParam("/camera_up_from_center", camera_up_from_center);
	
	// Initialize transforms
	tf2::Quaternion camera_forward_quat;
	camera_forward_quat.setRPY(0, 0, 0);
	camera_forward.header.stamp = ros::Time::now();
	camera_forward.header.frame_id = "base_footprint";
	camera_forward.child_frame_id = "camera_forward";
	camera_forward.transform.translation.x = -camera_forward_from_center;
	camera_forward.transform.translation.y = baseline_length/2.0;
	camera_forward.transform.translation.z = camera_forward_dist_from_ground;
	camera_forward.transform.rotation.x = camera_forward_quat.x();
	camera_forward.transform.rotation.y = camera_forward_quat.y();
	camera_forward.transform.rotation.z = camera_forward_quat.z();
	camera_forward.transform.rotation.w = camera_forward_quat.w();

	tf2::Quaternion camera_up_quat;
	camera_up_quat.setRPY(0, -M_PI/2.0, 0);
	camera_up.header.stamp = ros::Time::now();
	camera_up.header.frame_id = "base_footprint";
	camera_up.child_frame_id = "camera_up";
	camera_up.transform.translation.x = -camera_up_from_center;
	camera_up.transform.translation.y = -baseline_length/2.0;
	camera_up.transform.translation.z = camera_up_dist_from_ground;
	camera_up.transform.rotation.x = camera_up_quat.x();
	camera_up.transform.rotation.y = camera_up_quat.y();
	camera_up.transform.rotation.z = camera_up_quat.z();
	camera_up.transform.rotation.w = camera_up_quat.w();

	tf2::Quaternion camera_down_quat;
	camera_down_quat.setRPY(0, -M_PI/2.0 - 0.185, 0);
	camera_down.header.stamp = ros::Time::now();
	camera_down.header.frame_id = "arm_2";
	camera_down.child_frame_id = "camera_down";
	camera_down.transform.translation.x = -1.5*inches_to_meters;
	camera_down.transform.translation.y = 0;
	camera_down.transform.translation.z = 1.0*inches_to_meters;
	camera_down.transform.rotation.x = camera_down_quat.x();
	camera_down.transform.rotation.y = camera_down_quat.y();
	camera_down.transform.rotation.z = camera_down_quat.z();
	camera_down.transform.rotation.w = camera_down_quat.w();

	tf2::Quaternion arm_base_quat;
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

	ros::Rate rate(20.0);

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

		ros::spinOnce();
		rate.sleep();
	}

}