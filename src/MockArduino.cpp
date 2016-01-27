#include "ros/ros.h"

#include <serial/serial.h>
#include <sstream>

serial::Serial serial_port_;

int main(int argc, char **argv) {
	ros::init(argc, argv, "MockArduino");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	int motor_count, temp_int;
	std::ostringstream param_name;
	std::ostringstream pot_status;
	bool send_update;
	
	// Motor details
	pn.param<int>("motor_count", motor_count, 4);
	
	// Setup the hash that holds the current positions
	std::map<std::string,int> motors[motor_count];	
	
	for(int i = 0; i < motor_count; i++) {
		
		param_name.str("");
		param_name.clear();
		param_name << "motor_" << i << "_init";
		//ROS_INFO("Looking at [%s]", param_name.str().c_str());
		pn.param<int>(param_name.str(), temp_int, 0);
		motors[i]["status"] = temp_int;

		param_name.str("");
		param_name.clear();
		param_name << "motor_" << i << "_min";
		pn.param<int>(param_name.str(), temp_int, 100);
		motors[i]["min"] = temp_int;

		param_name.str("");
		param_name.clear();
		param_name << "motor_" << i << "_max";
		pn.param<int>(param_name.str(), temp_int, 200);
		motors[i]["max"] = temp_int;
	}

	// Read out what we got...
	ROS_INFO("Initialized MockArduino with Parameter values");
	for (int i = 0; i < motor_count; i++) {
		ROS_INFO("%i: %i, %i, %i", i, motors[i]["status"], motors[i]["min"], motors[i]["max"]);
	}
		
	ros::Rate loop_rate(10);
	std::string movement_request;
	std::string prev_movement_request = "1 100|2 100|3 100|4 100";
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");

	// Open Serial port for R/W 
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

	// Set initial values
	for (int i = 0; i < motor_count; i++) {
		pot_status << i << " " << motors[i]["status"] << "|";
	}

	int count=0;
	while(ros::ok()) {

		// Read Serial Port
		try{ 
			bool readable=serial_port_.waitReadable();
			bool send_status = false;
			bool looking = true;
			int motor, movement;
			if (readable) {
				movement_request = serial_port_.read(128); 

				while(looking) {
					
					if ( movement_request.find("A|") == 0) {
						ROS_DEBUG("Received status request from Bridge");
						send_status = true;
						movement_request = movement_request.erase(0,2);
					} else if ( movement_request.length() > 0 && movement_request.find(" ") >= 0 && movement_request.find("|") >= 0) {
						ROS_INFO("READ Movement Request from Bridge [%s]", movement_request.c_str());
                        motor = atoi(movement_request.substr(0, movement_request.find(" ")).c_str());
                        movement = atoi(movement_request.substr(movement_request.find(" ") + 1, movement_request.find("|")).c_str());
                        movement_request = movement_request.erase(0, movement_request.find("|") + 1);

						if ( movement <= motors[motor]["max"] && movement >= motors[motor]["min"] ) {
							ROS_INFO("Requested movement within threshold");
							motors[motor]["status"] = movement;
						} else {
							ROS_INFO("Requested movement outside of threshold, ignoring");
						}
					} else {
						looking = false;
					}
				}

				// Update status string
				pot_status.str("");
				pot_status.clear();
				for (int i = 0; i < motor_count; i++) {
					pot_status << i << " " << motors[i]["status"] << "|";
				}

				// Write back previous movement request...
				if (send_status == true ) {
					ROS_DEBUG("SEND status to Bridge [%s].", pot_status.str().c_str());
					serial_port_.write(pot_status.str());
				} 
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
