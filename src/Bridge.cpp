#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/MotorPosition.h>
#include <coconuts_common/ArmStatus.h>

#include <serial/serial.h>
#include <sstream>

#define DELIMITER "|"

serial::Serial serial_port_;

// Current motor status
// Hope we dont have more than 10...
int motors[10];

void movementCallback(const coconuts_common::ArmMovement::ConstPtr& msg){

	std::ostringstream request;

	ROS_INFO("Got Movment of type [%s].", msg->type.c_str());

	if ( msg->type == "ABSOLUTE" ) {

		for (int i = 0; i < msg->motor_positions.size(); i++) {
			int motor, position;
			motor = msg->motor_positions[i].motor;

			position = msg->motor_positions[i].position;

			request << motor << " " << position << "|";

			// Update our cached status 
			motors[motor] = position;
		}

	} else if ( msg->type == "RELATIVE" ) {

		for (int i = 0; i < msg->motor_positions.size(); i++) {
			int motor, position;
			motor = msg->motor_positions[i].motor;

			// Add relative request to current motor position 
			position = motors[motor] + msg->motor_positions[i].position;

			request << motor << " " << position << "|";

			// Update our cached status
			motors[motor] = position;
		}
		
	} else if ( msg->type == "POSE" ) {
		// TODO: Create a request using Pose data defined somewhere
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

	ros::init(argc, argv, "Bridge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	
   	// Maybe reduce this so we dont buffer commands?
	ros::Subscriber motor_sub = n.subscribe("motor_control",1, &movementCallback);
	ros::Publisher arm_status_pub = n.advertise<coconuts_common::ArmStatus>("/arm_status",1);
    	
   	ros::Rate loop_rate(2);

	// Get Parameters
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");

	// TODO Get Pre-fabs defined and included:)

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

		coconuts_common::ArmStatus arm_status;

		// Read Serial Port
		try{ 
			bool readable=serial_port_.waitReadable();
			if (readable) {
				std::string status = serial_port_.read(128); 
				ROS_DEBUG("READ Status from Arduino %s", status.c_str());
				
				// input is a repeating string of format: "X YYYY|"
				// if we find a "A|" we should assume its out of sequence and ignore
				if ( status.find("A|") == std::string::npos) {

					bool looking = true;
					coconuts_common::MotorPosition motor_position;

				    while (looking) {	
						if ( status.length() > 0 && status.find(" ") >= 0 && status.find("|") >= 0 ) {
							looking = true;
							motor_position.motor = atoi(status.substr(0, status.find(" ")).c_str());
							motor_position.position = atoi(status.substr(status.find(" ") + 1, status.find("|")).c_str());
							status = status.erase(0, status.find("|") + 1);
							arm_status.motor_positions.push_back(motor_position);

							// Copy values locally for relative movement
							motors[motor_position.motor] = motor_position.position;

							ROS_DEBUG("Found [%i %i|].", motor_position.motor, motor_position.position);
						} else {
							looking = false;
							ROS_DEBUG("Done looking [%s].", status.c_str());
						}
					}
					arm_status_pub.publish(arm_status);
				} else {
					ROS_INFO("Found out of sequence request [%s].", status.c_str());
				}
			}

			// Queue up next request for status, but only do it 1/5th of the times we send requests
			if ( count % 5 == 0 ) {
				serial_port_.write("A|");
			}

		} catch(std::exception& e){
			std::cerr<<e.what()<<std::endl;
		}

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
