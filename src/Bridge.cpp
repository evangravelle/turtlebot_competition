#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/MotorPosition.h>
#include <coconuts_common/ArmStatus.h>
#include <coconuts_common/SensorStatus.h>
#include <coconuts_common/SensorReading.h>

#include <serial/serial.h>
#include <sstream>

serial::Serial serial_port_;

// Current/cached motor positions
// Hope we dont have more than 10...
std::map<std::string,int> motors[10];
std::map<std::string,std::string> poses;

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

	ros::init(argc, argv, "Bridge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	std::ostringstream param_name;
	int motor_count, polling_frequency, sonar_polling_freq;
	
   	// Maybe reduce this so we dont buffer commands?
	ros::Subscriber motor_sub = n.subscribe("motor_control",1, &movementCallback);
	ros::Publisher arm_status_pub = n.advertise<coconuts_common::ArmStatus>("/arm_status",1);
	ros::Publisher sensor_status_pub = n.advertise<coconuts_common::SensorStatus>("/sensor_status",1);
    	

	// Get Parameters
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");
	pn.param<int>("motor_count", motor_count, 4);
	pn.param<int>("polling_frequency", polling_frequency, 20);
	pn.param<int>("sonar_polling_freq", sonar_polling_freq, 5);

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

		coconuts_common::ArmStatus arm_status;
		coconuts_common::SensorStatus sensor_status;

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
	                    coconuts_common::SensorReading sensor_reading;
						
						int loops = 0;
					    while (looking && loops < 10) {	
							if ( (status.length() > 0) && (status.find(" ") >= 0) && (status.find("|") >= 0) ) {
	                            // If Sensor Reading
	                            if ( (status.at(0)) == 's' ) {
	                                sensor_reading.sensor = atoi(status.substr(status.find("s") + 1, status.find(" ")).c_str() );
	                                sensor_reading.reading = atoi(status.substr(status.find(" ") + 1, status.find("|")).c_str());
	                                status = status.erase(0, status.find("|") + 1);
	                                sensor_status.sensor_readings.push_back(sensor_reading);
	                                // ROS_INFO("Sonar [%i %i]", sensor_reading.sensor, sensor_reading.reading);
	                            } else {
	                                motor_position.motor = atoi(status.substr(0, status.find(" ")).c_str());
	                                motor_position.position = atoi(status.substr(status.find(" ") + 1, status.find("|")).c_str());
	                                ROS_DEBUG("BEFORE: [%s].", status.c_str());
	                                status = status.erase(0, status.find("|") + 1);
	                                ROS_DEBUG("AFTER: [%s].", status.c_str());
	                                arm_status.motor_positions.push_back(motor_position);

	                                // Copy values locally for relative movement
	                                motors[motor_position.motor]["position"] = motor_position.position;

	                                ROS_DEBUG("STATUS Found [%i %i|].", motor_position.motor, motor_position.position);
	                            }
							} else {
								looking = false;
								ROS_DEBUG("Done looking [%s].", status.c_str());
							}
							loops++;
						}
						arm_status_pub.publish(arm_status);
						sensor_status_pub.publish(sensor_status);
					} else {
						ROS_INFO("Found out of sequence request [%s].", status.c_str());
					}
				}
			}

			// Queue up next request for status, but only do it 1/10th of the times we send requests
			if ( count % polling_frequency == 0 ) {
				serial_port_.write("A|");
			}
			if(count % sonar_polling_freq ==0){
				serial_port_.write("S|");
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
