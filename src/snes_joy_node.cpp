#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <coconuts_common/ArmMovement.h>
#include <coconuts_common/MotorPosition.h>

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
    double max_ang_vel_, max_lin_vel_;

public:

    snes_joy_handler() {
        joy_sub  = nodeh.subscribe<sensor_msgs::Joy>("/joy", 1, &snes_joy_handler::joy_receive, this);
        twist_pub = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        motor_pub = nodeh.advertise<coconuts_common::ArmMovement>("/motor_control", 1);

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


    void joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg) {

        coconuts_common::ArmMovement arm_movement;
		coconuts_common::MotorPosition motor_position;

        twist.linear.x = max_lin_vel_*joy_msg->axes.at(1);
        twist.angular.z = max_ang_vel_*joy_msg->axes.at(0);

		arm_movement.type = "RELATIVE";

        // RT Indicates a movement in the positive
        // LT indicates a movement in the negative
        // A,B,X,Y represent individual motors
        // Wont let you send up and down at same time. 

		// POSES:  Hold SELECT and A,B,X,T or START 

        // Assumes hardware device will be limiting movement past tolerances
        if(joy_msg->buttons.at(RT) && !joy_msg->buttons.at(LT)) {
            if(joy_msg->buttons.at(A)) {
                ROS_DEBUG("A going up");
				motor_position.motor = 0;
				motor_position.position = 10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(B)) {
                ROS_DEBUG("B going up");
				motor_position.motor = 1;
				motor_position.position = 10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(X)) {
                ROS_DEBUG("X going up");
				motor_position.motor = 2;
				motor_position.position = 10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(Y)) {
                ROS_DEBUG("Y going up");
				motor_position.motor = 3;
				motor_position.position = 10;
				arm_movement.motor_positions.push_back(motor_position);
            }
        }

        if(joy_msg->buttons.at(LT) && !joy_msg->buttons.at(RT)) {
            if(joy_msg->buttons.at(A)) {
                ROS_DEBUG("A going down");
				motor_position.motor = 0;
				motor_position.position = -10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(B)) {
                ROS_DEBUG("B going down");
				motor_position.motor = 1;
				motor_position.position = -10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(X)) {
                ROS_DEBUG("X going down");
				motor_position.motor = 2;
				motor_position.position = -10;
				arm_movement.motor_positions.push_back(motor_position);
            }
            else if(joy_msg->buttons.at(Y)) {
                ROS_DEBUG("Y going down");
				motor_position.motor = 3;
				motor_position.position = -10;
				arm_movement.motor_positions.push_back(motor_position);
            }
        }
            
        if(joy_msg->buttons.at(SELECT)) {
            ROS_DEBUG("SELECTing a POSE");

			arm_movement.type = "POSE";

            if(joy_msg->buttons.at(A)) {
				arm_movement.pose = "GRAB_BALL_OPEN";
            } else if(joy_msg->buttons.at(B)) {
				arm_movement.pose = "GRAB_BALL_CLOSE";
            } else if(joy_msg->buttons.at(X)) {
				arm_movement.pose = "DROP_BALL_OPEN";
            } else if(joy_msg->buttons.at(Y)) {
				arm_movement.pose = "DROP_BALL_CLOSE";
            } else if(joy_msg->buttons.at(START)) {
				arm_movement.pose = "SEARCH";
			}
        }

        if(joy_msg->buttons.at(START)) {
            ROS_DEBUG("START");
        }

	
		// Cut down on the chatter 
		if (arm_movement.type == "RELATIVE" &&
			arm_movement.motor_positions.size() > 0) {

				motor_pub.publish(arm_movement);

		} else if (arm_movement.type == "POSE" &&
				arm_movement.pose.length() > 0 ) {

				motor_pub.publish(arm_movement);
		}

		// Allways publish twist
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
