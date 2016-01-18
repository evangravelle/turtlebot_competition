#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "SimpleMover");
	ros::NodeHandle n:
    ros::Publisher trim_pot_pub = n.advertise<std_msgs::Float32MultiArray>("trim_pots",100);
    ros::Rate loop_rate(10);

	// Open Serial port for Reading

	int count=0;
	while(ros::ok()) {
		// Read Serial Port
		std_msgs::Float32MultiArray trim_pot_status;
		trim_pot_status.data = // Fake data
		ROS_INFO("%s",	


   
