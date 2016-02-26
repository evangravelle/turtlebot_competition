#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <states.h>

class mother_brain {

private:
    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber joy_sub;

	// Topics we Publis
    ros::Publisher twist_pub, motor_pub;

public:

    mother_brain() {
        //joy_sub  = nodeh.subscribe<sensor_msgs::Joy>("/joy", 1, &snes_joy_handler::joy_receive, this);
        //twist_pub = nodeh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        //motor_pub = nodeh.advertise<coconuts_common::ArmMovement>("/motor_control", 1);
    }


    void joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg) {
	}

    ~mother_brain() { }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mother_brain");
    mother_brain mother_brain_h;

    ROS_INFO("Mother Brain Started");
	
	// This structure would repeat in each callback
	// Logic that always runs
	// CASE statement specific logic
	// Explicit debugging 

    ros::spin();
}
