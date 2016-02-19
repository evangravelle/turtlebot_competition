#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <coconuts_common/ArmStatus.h>
#include <math.h>

double baseline_length = .248; // in inches
double camera_height = .406; // in inches
double camera_from_center = .038; // in y direction, in inches
double arm_base_y = .197;
double arm_base_z = .178;
double arm_1_length = .0965;
double arm_2_length = .0635;
double arm_3_length = .0508;

geometry_msgs::TransformStamped camera_forward, arm_base, arm_1, arm_2, arm_3, claw;

void armStatusCallback(const coconuts_common::ArmStatus::ConstPtr &arm_status) {

	//static tf2_ros::TransformBroadcaster tf_br_static;

	arm_base.transform.rotation.x = arm_status->motor_positions[0].position;
	arm_1.transform.rotation.x = arm_status->motor_positions[1].position;;
	arm_2.transform.rotation.x = arm_status->motor_positions[2].position;;
	arm_3.transform.rotation.x = arm_status->motor_positions[3].position;;
	claw.transform.rotation.x = arm_status->motor_positions[4].position;;

	//tf_br_static.sendTransform(arm_1);
	//tf_br_static.sendTransform(arm_2);
	//tf_br_static.sendTransform(arm_3);
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
	camera_forward.transform.translation.x = camera_from_center;
	camera_forward.transform.translation.y = baseline_length/2.0;
	camera_forward.transform.translation.z = camera_height;
	camera_forward.transform.rotation.x = 0;
	camera_forward.transform.rotation.y = 0;
	camera_forward.transform.rotation.z = 0;
	camera_forward.transform.rotation.w = 1;

	arm_base.header.stamp = ros::Time::now();
	arm_base.header.frame_id = "base_footprint";
	arm_base.child_frame_id = "arm_base";
	arm_base.transform.translation.x = 0;
	arm_base.transform.translation.y = arm_base_y;
	arm_base.transform.translation.z = arm_base_z;
	arm_base.transform.rotation.x = 0;
	arm_base.transform.rotation.y = 0;
	arm_base.transform.rotation.z = 0;
	arm_base.transform.rotation.w = 1;

	arm_1.header.stamp = ros::Time::now();
	arm_1.header.frame_id = "arm_base";
	arm_1.child_frame_id = "arm_1";
	arm_1.transform.translation.x = 0;
	arm_1.transform.translation.y = 0;
	arm_1.transform.translation.z = 0;
	arm_1.transform.rotation.x = M_PI/4.0;
	arm_1.transform.rotation.y = 0;
	arm_1.transform.rotation.z = 0;
	arm_1.transform.rotation.w = sqrt(1 - pow(cos(arm_1.transform.rotation.x),2));

	arm_2.header.stamp = ros::Time::now();
	arm_2.header.frame_id = "arm_1";
	arm_2.child_frame_id = "arm_2";
	arm_2.transform.translation.x = 0;
	arm_2.transform.translation.y = arm_1_length;
	arm_2.transform.translation.z = 0;
	arm_2.transform.rotation.x = -M_PI/4.0;
	arm_2.transform.rotation.y = 0;
	arm_2.transform.rotation.z = 0;
	arm_2.transform.rotation.w = sqrt(1 - pow(cos(arm_1.transform.rotation.x),2));

	arm_3.header.stamp = ros::Time::now();
	arm_3.header.frame_id = "arm_2";
	arm_3.child_frame_id = "arm_3";
	arm_3.transform.translation.x = 0;
	arm_3.transform.translation.y = arm_2_length;
	arm_3.transform.translation.z = 0;
	arm_3.transform.rotation.x = -M_PI/4.0;
	arm_3.transform.rotation.y = 0;
	arm_3.transform.rotation.z = 0;
	arm_3.transform.rotation.w = sqrt(1 - pow(cos(arm_1.transform.rotation.x),2));

	claw.header.stamp = ros::Time::now();
	claw.header.frame_id = "arm_3";
	claw.child_frame_id = "claw";
	claw.transform.translation.x = 0;
	claw.transform.translation.y = arm_3_length;
	claw.transform.translation.z = 0;
	claw.transform.rotation.x = -M_PI/4.0;
	claw.transform.rotation.y = 0;
	claw.transform.rotation.z = 0;
	claw.transform.rotation.w = sqrt(1 - pow(cos(arm_1.transform.rotation.x),2));


	while (ros::ok()){

		camera_forward.header.stamp = ros::Time::now();
		arm_base.header.stamp = ros::Time::now();
		arm_1.header.stamp = ros::Time::now();
		arm_2.header.stamp = ros::Time::now();
		arm_3.header.stamp = ros::Time::now();
		claw.header.stamp = ros::Time::now();

		tf_br.sendTransform(camera_forward);
		tf_br.sendTransform(arm_base);
		tf_br.sendTransform(arm_1);
		tf_br.sendTransform(arm_2);
		tf_br.sendTransform(arm_3);
		tf_br.sendTransform(claw);

		rate.sleep();
	}

}