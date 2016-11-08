#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/MotorPosition.h>
#include <coconuts_common/ArmStatus.h>

#include <serial/serial.h>
#include <sstream>

serial::Serial serial_port_;

// Current/cached motor positions
// Hope we dont have more than 10...
std::map<std::string,int> motors[10];
std::map<std::string,std::string> poses;

int main(int argc, char **argv) {

	ros::init(argc, argv, "poll_arm");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	int polling_frequency;
	
   	// Maybe reduce this so we dont buffer commands?
	ros::Publisher arm_status_pub = n.advertise<coconuts_common::ArmStatus>("/arm_status",1);
    	

	// Get Parameters
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");
	pn.param<int>("polling_frequency", polling_frequency, 1);

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

		coconuts_common::ArmStatus arm_status;

		// TODO:
		// Parsing logic is too fragile, can be garbage till "hello"
		// possibly some carrage returns, clean those out first
		// READ until hello
		// cleanse anything other than numbers, space, pipe
		// then try to find the values
		// some loop count so we dont go on forever
		// Read Serial Port
		try{ 
			bool readable=serial_port_.waitReadable();
			if (readable) {
				std::string status = serial_port_.read(128); 
				ROS_INFO("READ Status from Arduino %s", status.c_str());
				
				if (status.find("hello") != std::string::npos) {
					ROS_INFO("Its Adele, turn that shit off");
				} else {
				// input is a repeating string of format: "X YYYY|"
				// if we find a "A|" we should assume its out of sequence and ignore
				if ( status.length() > 0 && status.find("A|") == std::string::npos && status.find(" ") >= 0 && status.find("|") >= 0 ) {

					bool looking = true;
					coconuts_common::MotorPosition motor_position;
					
					int loops = 0;
				    while (looking && loops < 10) {	
						if ( (status.length() > 0) && (status.find(" ") >= 0) && (status.find("|") >= 0) ) {
							motor_position.motor = atoi(status.substr(0, status.find(" ")).c_str());
							motor_position.position = atoi(status.substr(status.find(" ") + 1, status.find("|")).c_str());
							ROS_DEBUG("BEFORE: [%s].", status.c_str());
							status = status.erase(0, status.find("|") + 1);
							ROS_DEBUG("AFTER: [%s].", status.c_str());
							arm_status.motor_positions.push_back(motor_position);

							// Copy values locally for relative movement
							motors[motor_position.motor]["position"] = motor_position.position;

							ROS_DEBUG("STATUS Found [%i %i|].", motor_position.motor, motor_position.position);
						} else {
							looking = false;
							ROS_DEBUG("Done looking [%s].", status.c_str());
						}
						loops++;
					}
					arm_status_pub.publish(arm_status);
				} else {
					ROS_INFO("Found out of sequence request [%s].", status.c_str());
				}
			}
			}

			// Queue up next request for status, but only do it 1/Nth of the times we send requests
			if ( count % polling_frequency == 0 ) {
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
