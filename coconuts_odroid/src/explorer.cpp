#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <states.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/SensorStatus.h>


// Ugly, but this works
int behavior_state_;
int prev_behavior_state_;
int behavior_sub_state_;

class explorer {

private:

    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber 
        control_state_sub,
        sensor_status_sub;

	// Topics we Publish
    ros::Publisher
        cmd_vel_pub;

    int movement;
    int max_movements;
    bool left_obstacle;
    bool right_obstacle;
    bool center_obstacle;

public:

    explorer() {
        // Initialise Globals
        behavior_state_ = INIT;
        prev_behavior_state_ = INIT;
        behavior_sub_state_ = DEFAULT_SUB_STATE;
        movement = 0;
        max_movements = 10;
        left_obstacle = false;
        right_obstacle = false;
        center_obstacle = false;
        
        // Subscriptions
        control_state_sub  = nodeh.subscribe<coconuts_common::ControlState>("/control_state", 1, &explorer::control_state_receive, this);
        sensor_status_sub  = nodeh.subscribe<coconuts_common::SensorStatus>("/sensor_status", 1, &explorer::sensor_status_receive, this);

        // Publishers
        cmd_vel_pub = nodeh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);


    }


    void control_state_receive(const coconuts_common::ControlState::ConstPtr& control_msg) {

        behavior_state_ = control_msg->state;
        behavior_sub_state_ = control_msg->sub_state;

	}

    void sensor_status_receive(const coconuts_common::SensorStatus::ConstPtr& sensor_msg) {

        // TODO define these magic numbers and parameterize the reading value
        for (int i = 0; i < sensor_msg->sensor_readings.size(); i++ ) {
            //ROS_INFO("Explorer: Looking at sensor [%d] reading [%d].", sensor_msg->sensor_readings[i].sensor, sensor_msg->sensor_readings[i].reading);

            switch (sensor_msg->sensor_readings[i].sensor) {
                case 0:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 5) {
                        center_obstacle = true;
                    } else {
                        center_obstacle = false;
                    }
                    break;

                case 1:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 40) {
                        right_obstacle = true;
                    } else {
                        right_obstacle = false;
                    }
                    break;

                case 2:
                    if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 40) {
                        left_obstacle = true;
                    } else {
                        left_obstacle = false;
                    }
                    break;
            
                default:
                    break;
            }
        }
	}

    void exploreOnce() {

        movement++;
        geometry_msgs::Twist search_twist;
        // Bone dumb obstacle avoidance
        // Go forward unless we run into something
        search_twist.linear.x = 0.1;
        search_twist.angular.z = 0.0;
        if (left_obstacle && right_obstacle) {
            search_twist.linear.x = -0.3; // Go Backwards
        } else {
            if (left_obstacle) {
                search_twist.angular.z = -1.0; // rotate right
                search_twist.linear.x = 0.0;
            }
            else if (right_obstacle) {
                search_twist.angular.z = 1.0; // rotate left
                search_twist.linear.x = 0.0;
            }
        }
        // Do we care about center here?

        // Drive the bot around 
        // TODO:  This needs to drive in some search patern
        // TODO:  Look at some better sutied packages: http://wiki.ros.org/hector_navigation
        //
        cmd_vel_pub.publish(search_twist);

        if (movement >= max_movements) movement = 0;
    }
        
    ~explorer() { }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explorer");
    explorer explorer_h;

    ROS_INFO("Explorer Started");

    while(ros::ok()) {

        // TODO :  start/stop should probably be services, not messages
        // as for messages there isnt a way to properly interrupt processing
	
        // If there was a state change
        if (prev_behavior_state_ != behavior_state_) {
            ROS_INFO("Explorer: State changed from [%d] to [%d].", prev_behavior_state_, behavior_state_);
            prev_behavior_state_ = behavior_state_;

            // Send notices..
            if (behavior_state_ == FIND_GOAL || behavior_state_ == FIND_BALL) {
                ROS_INFO("Explorer: Starting to Explore [%d].", behavior_state_);
            }
        }
        
        switch ( behavior_state_ ) {

            case FIND_GOAL:
                explorer_h.exploreOnce();
                break;

            case FIND_BALL:
                explorer_h.exploreOnce();
                break;

            default: 
                break;
        }
        ros::spinOnce();
    }
}


/*
 * Default Switch Code
 *
 */

/*
    switch ( behavior_state_ ) {

        case INIT:
            //
            break;

        case MANUAL:
            //
            break;

        case START:
            //
            break;

        case END:
            //
            break;

        case CONFIG:
            //
            break;

        case FIND_GOAL:
            //
            break;

        case MOVE_TO_GOAL:
            //
            break;

        case FIND_BALL:
            //
            break;

        case MOVE_TO_BALL:
            //
            break;

        case PICK_UP_BALL:
            //
            break;

        case DROP_BALL:
            //
            break;

        default: 
            ROS_ERROR("Default called on case stament.  Unexpected state [%i] in mother_brain:main().", behavior_state_);

    }

*/
