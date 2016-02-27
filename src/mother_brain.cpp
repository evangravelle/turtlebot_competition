#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <states.h>
#include <coconuts_common/ControlState.h>


// Ugly, but this works
int behavior_state_;
int behavior_sub_state_;

class mother_brain {

private:

    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber 
        control_override_sub;

	// Topics we Publish
    ros::Publisher
        control_state_pub;

public:

    mother_brain() {
        behavior_state_ = INIT;
        behavior_sub_state_ = DEFAULT_SUB_STATE;
        
        control_override_sub  = nodeh.subscribe<coconuts_common::ControlState>("/control_state_override", 1, &mother_brain::control_override_receive, this);
        control_state_pub = nodeh.advertise<coconuts_common::ControlState>("/control_state", 1);
    }


    void control_override_receive(const coconuts_common::ControlState::ConstPtr& control_msg) {

        behavior_state_ = control_msg->state;

	}

    void publish_state() {
        coconuts_common::ControlState current_state;
        current_state.state = behavior_state_;
        current_state.sub_state = behavior_sub_state_;
        control_state_pub.publish(current_state);
    }


    ~mother_brain() { }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mother_brain");
    mother_brain mother_brain_h;

    ROS_INFO("Mother Brain Started");

    while(ros::ok()) {
	
        // This structure would repeat in each callback
        // Logic that always runs
        // CASE statement specific logic
        // Explicit debugging 

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

        // Publish Current state and sub_state
        //
        mother_brain_h.publish_state();

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
