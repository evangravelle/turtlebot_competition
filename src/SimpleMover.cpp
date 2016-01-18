#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <serial/serial.h>
#include <sstream>

std::string port_name_ = "/dev/pts/27";
serial::Serial serial_port_;

void movementCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Recieved Movement: Linear: %f, %f, %f  Angular: %f, %f, %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
	std::ostringstream to_send;
	// use linear X to represent the motor number
	// user angular Z to represent the degrees to turn
    // send movement
	to_send << msg->linear.x << " " << msg->angular.z << std::endl;
	
	std::cout<<"Sending Movement: "<<to_send.str()<<std::endl;
	try {
		serial_port_.write(to_send.str());
	} catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "SimpleMover");
	ros::NodeHandle n;
        // Maybe reduce this so we dont buffer commands?
	ros::Subscriber sub = n.subscribe("cmd_vel",100, movementCallback);
    ros::Rate loop_rate(10);

	std::string trim_pot_status;

	// Open Serial port for Reading
	try {
			serial_port_.setPort(port_name_);
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
