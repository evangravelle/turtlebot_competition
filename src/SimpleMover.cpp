#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <serial/serial.h>
#include <sstream>

serial::Serial serial_port_;

double kp,ki,kd;

void pidGainCallback(const geometry_msgs::Vector3::ConstPtr& gainPtr)
{
    kp = (double) gainPtr -> x;
    ki = (double) gainPtr -> y;
    kd = (double) gainPtr -> z;
} 

void movementCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Received Movement: %s", msg->data.c_str());
	std::ostringstream to_send;
	to_send << msg->data << " " << kp << " " << ki << " "<< kd;
	ROS_INFO("Sending Movement (w/PID): %s",to_send.str().c_str());
	//std::cout<<"Sending Movement: "<<to_send.str()<<std::endl;
	try {
		serial_port_.write(to_send.str());
	} catch(std::exception& e){
		std::cerr<<e.what()<<std::endl;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "SimpleMover");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	std::string port_name;
	
        // Maybe reduce this so we dont buffer commands?
	ros::Subscriber sub = n.subscribe("motor_control",100, movementCallback);
    	ros::Subscriber pidGainSub = n.subscribe<geometry_msgs::Vector3>("/pid_gain", 1, pidGainCallback);
    	
    	ros::Rate loop_rate(10);
	std::string trim_pot_status;
	pn.param<std::string>("port", port_name, "/dev/ttyUSB0");

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
		// Read Serial Port
		try{ 
        		bool readable=serial_port_.waitReadable();
        		if (readable) {
					trim_pot_status = serial_port_.read(128); 
					ROS_INFO("Read Status %s", trim_pot_status.c_str());
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
