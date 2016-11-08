#include <ros/ros.h>
#include <coconuts_master/ConfigStereoCameraAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<coconuts_master::ConfigStereoCameraAction> ConfigStereoServer;

void execute(const coconuts_master::ConfigStereoCameraGoalConstPtr& goal, ConfigStereoServer* as) {
	ROS_INFO("Executing Goal");
	as->setSucceeded();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "action_server_example");
	ros::NodeHandle n;
	ConfigStereoServer server(n, "config_stereo_camera", boost::bind(&execute, _1, &server), false);
	server.start();
	ROS_INFO("Setting up server");
	ros::spin();
	return 0;
}
