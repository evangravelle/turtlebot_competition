#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/ArmStatus.h>
#include <coconuts_common/MotorPosition.h>

#include <serial/serial.h>
#include <sstream>

serial::Serial serial_port_;

// Current/cached motor positions
// Hope we dont have more than 10...
std::map<std::string,int> motors[10];
std::map<std::string,std::string> poses;

void statusCallback(const coconuts_common::ArmStatus::ConstPtr& msg){

	int position, motor;

	//Update cached position with real positions
	for ( int i = 0; i < msg->motor_positions.size(); i++) {
	
			position = msg->motor_positions[i].position;
			motor = msg->motor_positions[i].motor;
			motors[motor]["position"] = position;
	}

}

void movementCallback(const coconuts_common::ArmMovement::ConstPtr& msg){

	std::ostringstream request;

	ROS_INFO("Got Movment of type [%s].", msg->type.c_str());

	if ( msg->type == "ABSOLUTE" ) {

		for (int i = 0; i < msg->motor_positions.size(); i++) {
			int motor, position;
			motor = msg->motor_positions[i].motor;

			position = msg->motor_positions[i].position;

			request << motor << " " << position << "|";

			// Update our cached position 
			motors[motor]["position"] = position;
		}

	} else if ( msg->type == "RELATIVE" ) {

		// Note actual requested values are ignored, intead, moves by 
		// some default increments which depend on the motor in use
		// Only the sign of the requested movement is used

		for (int i = 0; i < msg->motor_positions.size(); i++) {
			int motor, position;
			motor = msg->motor_positions[i].motor;

			// Add relative request to current motor position 
			if ( msg->motor_positions[i].position >= 0 ) {
				if ( motors[motor]["inverted"] ) {
					position = motors[motor]["position"] - motors[motor]["increment"];
				} else {
					position = motors[motor]["position"] + motors[motor]["increment"];
				}
			} else {
				if ( motors[motor]["inverted"] ) {
					position = motors[motor]["position"] + motors[motor]["increment"];
				} else {
					position = motors[motor]["position"] - motors[motor]["increment"];
				}
			}

			request << motor << " " << position << "|";

			// Update our cached status
			motors[motor]["position"] = position;
		}
		
	} else if ( msg->type == "POSE" ) {
		ROS_INFO("Got Pose [%s].", msg->pose.c_str());
		request << poses[msg->pose.c_str()];
	}

	ROS_INFO("SEND Request to Arduino [%s].", request.str().c_str());

	if ( request.str().length() > 0 ) {
		try {
			serial_port_.write(request.str());
		} catch(std::exception& e){
			std::cerr<<e.what()<<std::endl;
		}
	} else {
		ROS_INFO("Nothing to Send");
	}

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "control_arm");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	std::ostringstream param_name;
	int motor_count;
	
   	// Maybe reduce this so we dont buffer commands?
	ros::Subscriber motor_control_sub = n.subscribe("motor_control",1, &movementCallback);

	// Do we need to keep a local copy of current positions??

	// Get Parameters
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");
	pn.param<int>("motor_count", motor_count, 4);

	for(int i = 0; i < motor_count; i++) {

		param_name.str("");
		param_name.clear();
		param_name << "motor_" << i << "_inverted";
		pn.param<int>(param_name.str(), motors[i]["inverted"], 0);

		param_name.str("");
		param_name.clear();
		param_name << "motor_" << i << "_increment";
		pn.param<int>(param_name.str(), motors[i]["increment"], 10);

		// Defaulting this until we read first status
		motors[i]["position"] = 200;
	}

	// TODO Get Pre-fabs defined and included:)
	// Canned Poses
	pn.param<std::string>("SEARCH", poses["SEARCH"], "UNDEF");
	pn.param<std::string>("GRAB_BALL_OPEN", poses["GRAB_BALL_OPEN"], "UNDEF");
	pn.param<std::string>("GRAB_BALL_CLOSE", poses["GRAB_BALL_CLOSE"],"UNDEF");
	pn.param<std::string>("DROP_BALL_OPEN", poses["DROP_BALL_OPEN"], "UNDEF");
	pn.param<std::string>("DROP_BALL_CLOSE", poses["DROP_BALL_CLOSE"], "UNDEF");

   	ros::Rate loop_rate(10);

	// Open Serial port for Reading
	try {
		serial_port_.setPort(port_name);
		serial_port_.setBaudrate(115200);
		serial::Timeout T = serial::Timeout::simpleTimeout(100);
		serial_port_.setTimeout(T);
		serial_port_.open();
	} catch (std::exception & e) {
		std::cerr<<"Error Opening Serial port: " <<e.what()<<std::endl;
		return(-1);
	}

	int count=0;

	while(ros::ok()) {

		// Execute callbacks... 
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	try {
		serial_port_.close();
	} catch (std::exception & e) {
		std::cerr<<"Error Closing Serial port: " <<e.what()<<std::endl;
		return(-1);
	}

	return 0;
}
