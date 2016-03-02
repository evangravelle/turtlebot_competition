#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <states.h>
#include <coconuts_common/ControlState.h>


// Ugly, but this works
int behavior_state_;
int prev_behavior_state_;
int behavior_sub_state_;

class mother_brain {

private:

    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber 
        control_override_sub;

	// Topics we Publish
    ros::Publisher
        control_state_pub,
        goal_pose_pub,
        cmd_vel_pub;

    tf::TransformListener ball_found_listener;

public:

    mother_brain() {
        // Initialise Globals
        behavior_state_ = INIT;
        prev_behavior_state_ = INIT;
        behavior_sub_state_ = DEFAULT_SUB_STATE;
        
        // Subscriptions
        control_override_sub  = nodeh.subscribe<coconuts_common::ControlState>("/control_state_override", 1, &mother_brain::control_override_receive, this);

        // Publishers
        control_state_pub = nodeh.advertise<coconuts_common::ControlState>("/control_state", 1);
        goal_pose_pub = nodeh.advertise<geometry_msgs::TransformStamped>("/goal_pose", 1);
        cmd_vel_pub = nodeh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);


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

    void find_ball() {

        if (behavior_state_ == FIND_BALL) {
            tf::StampedTransform ball_found_transform;

            // Drive the bot around 
            // TODO:  This needs to drive in some search patern
            // and avoid obstacles - shoud potentially be another node that
            // only runs in this state (FIND_BALL, SEARCH_FOR_BALL)
            behavior_sub_state_ = SEARCH_FOR_BALL;
            // Move forward a bit
            // rotate 90
            geometry_msgs::Twist search_twist;
            search_twist.angular.z = 2;
            search_twist.linear.x = 10;
            cmd_vel_pub.publish(search_twist);


            // Wait for a sign indicating we've found a ball
            ros::Time now = ros::Time::now();
            try {
                ball_found_listener.waitForTransform("/camera_forward", "/ball_forward", now, ros::Duration(3.0));
                ball_found_listener.lookupTransform("/camera_forward", "/ball_forward", now, ball_found_transform);
                // set states 
                behavior_state_ = MOVE_TO_BALL;
                behavior_sub_state_ = BALL_FOUND;
                // publish transform to goal_pose
                ROS_INFO("Ball found, moving to ball and changing state to MOVE_TO_BALL");
                publish_goal(ball_found_transform);

            } catch (tf::TransformException ex){
                ROS_INFO("Caught exception waiting for FIND_BALL transform");
                ROS_ERROR("%s",ex.what());
            }
        }
    }
        

    void publish_goal(tf::StampedTransform transform) {
        // How do we confert the StampedTransform to a geometry_msgs::PoseStamped
        geometry_msgs::TransformStamped msg;
        tf::transformStampedTFToMsg(transform, msg);
        goal_pose_pub.publish(msg);
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

        if (prev_behavior_state_ != behavior_state_) {
            ROS_INFO("State changed from [%d] to [%d].", prev_behavior_state_, behavior_state_);
            prev_behavior_state_ = behavior_state_;
        }

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

                // Changing state here causes Search node to control the Turtlebot
                //
                behavior_sub_state_ = SEARCH_FOR_BALL;

                mother_brain_h.find_ball();

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
