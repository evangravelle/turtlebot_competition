#include <ros/ros.h>
#include <coconuts_common/ArmStatus.h>
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/MotorPosition.h>
#include <math.h>

int main(int argc, char **argv) {

	ros::init(argc, argv, "arm_pub_test");
	ros::NodeHandle nh;
	ros::Publisher arm_status_pub = nh.advertise<coconuts_common::ArmStatus>("/arm_status",1);

	coconuts_common::MotorPosition motor_position_0, motor_position_1, motor_position_2;

	motor_position_0.motor = 0;
	motor_position_1.motor = 1;
	motor_position_2.motor = 2;
	double seconds_per_revolution = 1/(2*M_PI);
	double begin_time = ros::Time::now().toSec();

	ros::Rate rate(5.0);
	while (ros::ok()){

		coconuts_common::ArmStatus arm_status;

		double current_time = ros::Time::now().toSec() - begin_time;
		motor_position_0.position = 250.0*sin(current_time) + 250.0;
		motor_position_1.position = 250.0*sin(current_time + 2.0) + 250.0;
		motor_position_2.position = 250;

		arm_status.motor_positions.push_back(motor_position_2);
		arm_status.motor_positions.push_back(motor_position_1);
		arm_status.motor_positions.push_back(motor_position_0);

		arm_status.header.stamp = ros::Time::now();
		arm_status_pub.publish(arm_status);
		rate.sleep();
	}

}