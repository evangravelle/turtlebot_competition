#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#define A 0
#define B 1
#define X 2
#define Y 3
#define LT 4
#define RT 5
#define SELECT 6
#define START 7


class snes_joy_handler {

private:
    ros::NodeHandle nodeh;
    // This works in other code .. dont know why its not working here
    //ros::NodeHandle pn("\~");

    ros::Subscriber joy_sub;
    ros::Publisher twist_pub, motor_pub;
    geometry_msgs::Twist twist;
    std_msgs::String motor;
    double max_ang_vel_, max_lin_vel_;

public:

    snes_joy_handler() {
        joy_sub  = nodeh.subscribe<sensor_msgs::Joy>("/joy", 1, &snes_joy_handler::joy_receive, this);
        twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        motor_pub = nodeh.advertise<std_msgs::String>("/motor_control", 1);

        //pn.param<double>("max_ang_vel", max_ang_vel_, 1.0);
        //pn.param<double>("max_lin_vel", max_lin_vel_, 0.25);
        // pn not working ... use defaults
        max_ang_vel_ = 1.0;
        max_lin_vel_ = 0.25;

        twist.linear.x=0;
        twist.linear.y=0;
        twist.linear.z=0;
        twist.angular.x=0;
        twist.angular.y=0;
        twist.angular.z=0;
    }

    // TODO:
    void joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg) {

        twist.linear.x = max_lin_vel_*joy_msg->axes.at(1);
        twist.angular.z = max_ang_vel_*joy_msg->axes.at(0);
		bool send_motor = false;
		motor.data = "";

        // RT Indicates a movement in the positive
        // LT indicates a movement in the negative
        // A,B,X,Y represent individual motors
        // Wont let you send up and down at same time. 
        // Assumes hardware device will be limiting movement past tolerances
        if(joy_msg->buttons.at(RT) && !joy_msg->buttons.at(LT)) {
            if(joy_msg->buttons.at(A)) {
                ROS_INFO("A going up");
				send_motor = true;
				motor.data = "0 10";
            }
            else if(joy_msg->buttons.at(B)) {
                ROS_INFO("B going up");
				send_motor = true;
				motor.data = "1 10";	
            }
            else if(joy_msg->buttons.at(X)) {
                ROS_INFO("X going up");
				send_motor = true;
				motor.data = "2 10";	
            }
            else if(joy_msg->buttons.at(Y)) {
                ROS_INFO("Y going up");
				send_motor = true;
				motor.data = "3 10";	
            }
        }

        if(joy_msg->buttons.at(LT) && !joy_msg->buttons.at(RT)) {
            if(joy_msg->buttons.at(A)) {
                ROS_INFO("A going down");
				send_motor = true;
				motor.data = "0 -10";	
            }
            else if(joy_msg->buttons.at(B)) {
                ROS_INFO("B going down");
				send_motor = true;
				motor.data = "1 -10";	
            }
            else if(joy_msg->buttons.at(X)) {
                ROS_INFO("X going down");
				send_motor = true;
				motor.data = "2 -10";	
            }
            else if(joy_msg->buttons.at(Y)) {
                ROS_INFO("Y going down");
				send_motor = true;
				motor.data = "3 -10";	
            }
        }
            
        if(joy_msg->buttons.at(SELECT)) {
            ROS_INFO("SELECT");
        }

        if(joy_msg->buttons.at(START)) {
            ROS_INFO("START");
        }

	
		if (send_motor == true) {
			motor_pub.publish(motor);
		}

		twist_pub.publish(twist);
	}

    ~snes_joy_handler() { }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "snes_joy_node");
    snes_joy_handler joy_h;

    ROS_INFO("SNES Joy Handler Started");

    ros::spin();
}
