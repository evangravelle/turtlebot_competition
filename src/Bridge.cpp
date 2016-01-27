#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <coconuts_odroid/Movement.h>

#include <serial/serial.h>
#include <sstream>

#define DELIMITER "|"

serial::Serial serial_port_;

int motors[10];


void movementCallback(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("RECV Absolute Movement Request from client: %s", msg->data.c_str());
	ROS_INFO("SEND Absolute Position to Arduino: %s",msg->data.c_str());
	try {
		serial_port_.write(msg->data);
	} catch(std::exception& e){
		std::cerr<<e.what()<<std::endl;
	}

	int motor, movement;
	std::ostringstream request;
	ROS_INFO("RECV Relative Movement Request from client: %s", msg->data.c_str());
	ROS_INFO("Calculating absolute position from relative change");
	// Get motor and adjustment

	motor = atoi(msg->data.substr(0, msg->data.find(" ")).c_str());
	movement = atoi(msg->data.substr(msg->data.find(" ") + 1, msg->data.find("|")).c_str());

	// Get Cached Positions, udpate and send
	movement += motors[motor];

	request << motor << " " << movement << "|";
	
	ROS_INFO("SEND Absolute Position to Arduino: [%s]",request.str().c_str());
	try {
		serial_port_.write(request.str());
	} catch(std::exception& e){
		std::cerr<<e.what()<<std::endl;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "Bridge");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	
   	// Maybe reduce this so we dont buffer commands?
	ros::Subscriber motor_sub = n.subscribe("motor_control",100, &movementCallback);
	ros::Publisher pot_status_pub = n.advertise<std_msgs::String>("/pot_status",1);
    	
   	ros::Rate loop_rate(10);
	std::string trim_pot_status;
	std_msgs::String tps;
	int motor, status;
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");
	pn.param<double>("kp", kp_, 0.0);
	pn.param<double>("ki", ki_, 0.0);
	pn.param<double>("kd", kd_, 0.0);

	// Open Serial port for Reading
	try {
			serial_port_.setPort(port_name);
			serial_port_.setBaudrate(115200);
			serial::Timeout T = serial::Timeout::simpleTimeout(100);
			serial_port_.setTimeout(T);
			serial_port_.open();
		
			// Send request for initial status
			serial_port_.write("A|");
	
	} catch (std::exception & e) {
		std::cerr<<"Error Opening Serial port: " <<e.what()<<std::endl;
		return(-1);
	}

	int count=0;
	while(ros::ok()) {
		// Read Serial Port
		try{ 
        		bool readable=serial_port_.waitReadable();
        		if (readable) {
					trim_pot_status = serial_port_.read(128); 
					ROS_INFO("READ Status from Arduino %s", trim_pot_status.c_str());

					tps.data = trim_pot_status;
					// update current values
					// TODO: This is very fragile....
					int count = 0;
					while(trim_pot_status.length() > 0 && count < 10) {
						motor = atoi(trim_pot_status.substr(0, trim_pot_status.find(" ")).c_str());
						status = atoi(trim_pot_status.substr(trim_pot_status.find(" ") + 1, trim_pot_status.find("|")).c_str());
						trim_pot_status.erase(0, trim_pot_status.find("|"));
						motors[motor] = status;
						count++;
					}
					
					// Publish response back out
					pot_status_pub.publish(tps);
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
