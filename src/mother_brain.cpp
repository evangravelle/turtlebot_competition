#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <states.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/ArmMovement.h>


// Ugly, but this works
int behavior_state_;
int prev_behavior_state_;
int behavior_sub_state_;

class mother_brain {

private:

    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber 
        control_override_sub,
        control_substate_sub,
        detect_ball_forward_sub;

	// Topics we Publish
    ros::Publisher
        control_state_pub,
        goal_pose_pub,
        cmd_vel_pub,
        motor_control_pub;

    tf::TransformListener ball_found_listener;

public:

    mother_brain() {
        // Initialise Globals
        behavior_state_ = INIT;
        prev_behavior_state_ = INIT;
        behavior_sub_state_ = DEFAULT_SUB_STATE;
        
        // Subscriptions
        control_override_sub  = nodeh.subscribe<coconuts_common::ControlState>("/control_state_override", 1, &mother_brain::control_override_receive, this);
        control_substate_sub = nodeh.subscribe<coconuts_common::ControlState>("/control_substate", 1, &mother_brain::control_substate_receive, this);
        detect_ball_forward_sub  = nodeh.subscribe<geometry_msgs::Point>("/detect_ball_forward/ball_pixel", 1, &mother_brain::find_ball_callback, this);

        // Publishers
        control_state_pub = nodeh.advertise<coconuts_common::ControlState>("/control_state", 10);
        goal_pose_pub = nodeh.advertise<geometry_msgs::TransformStamped>("/goal_pose", 1);
        cmd_vel_pub = nodeh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        motor_control_pub = nodeh.advertise<coconuts_common::ArmMovement>("/motor_control", 1);


    }


    void control_override_receive(const coconuts_common::ControlState::ConstPtr& control_msg) {

        behavior_state_ = control_msg->state;
        behavior_sub_state_ = control_msg->sub_state;
        ROS_INFO("Mother Brain: Forced control state to [%d].", behavior_state_);

	}

    void control_substate_receive(const coconuts_common::ControlState::ConstPtr& control_msg) {

        behavior_sub_state_ = control_msg->sub_state;
        ROS_INFO("Mother Brain: Forced sub state to [%d].", behavior_sub_state_);

    }

    void publish_state() {
        coconuts_common::ControlState current_state;
        current_state.state = behavior_state_;
        current_state.sub_state = behavior_sub_state_;
        control_state_pub.publish(current_state);
    }


    void move_to_ball() {

        if (behavior_state_ == MOVE_TO_BALL) {
            // Need to know:
            // 1.  when we're at the ball
            // 2.  When we're ready to pick up the ball
            // 3.  When do we give up?
            if (behavior_sub_state_ == MOVING_TO_BALL) { //  still getting there.
                //ROS_INFO("Mother Brain: Moving To Ball.");
            }

            // AT_BALL set by external node
            // If we are ready
            if (behavior_sub_state_ == AT_BALL) {
                ROS_INFO("Mother Brain: At Ball.");
                behavior_state_ = PICK_UP_BALL;
                behavior_sub_state_ = DEFAULT_SUB_STATE;
            }

            if (false) {
                // Failure, go back to FIND_BALL
                ROS_INFO("Mother Brain: Find Ball FAILED.");
                behavior_state_ = FIND_BALL;
                behavior_sub_state_ = DEFAULT_SUB_STATE;
            }
        }

    }

    void find_ball_callback(const geometry_msgs::Point::ConstPtr& msg) {

        if (behavior_state_ == FIND_BALL) {
                behavior_state_ = MOVE_TO_BALL;
                behavior_sub_state_ = MOVING_TO_BALL;
                // we just need to stop looking
                ROS_INFO("Ball found, moving to ball and changing state to MOVE_TO_BALL");
        }
    }

    void find_ball() {

        if (behavior_state_ == FIND_BALL) {
            tf::StampedTransform ball_found_transform;

            // Wait for a sign indicating we've found a ball
            ros::Time now = ros::Time::now();
            try {
                ball_found_listener.waitForTransform("/camera_forward", "/ball_forward", now, ros::Duration(3.0));
                ball_found_listener.lookupTransform("/camera_forward", "/ball_forward", now, ball_found_transform);
                // set states 
                behavior_state_ = MOVE_TO_BALL;
                behavior_sub_state_ = MOVING_TO_BALL;
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

    void arm_grabBallOpen() {
        ROS_INFO("Mother Brain: Moving arm to grab ball open.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "GRAB_BALL_OPEN";
        positionArm(arm_movement);
    }

    void arm_grabBallClose() {
        ROS_INFO("Mother Brain: Moving arm to grab ball close.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "GRAB_BALL_CLOSE";
        positionArm(arm_movement);
    }

    void arm_dropBallOpen() {
        ROS_INFO("Mother Brain: Moving arm to drop ball open.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "DROP_BALL_OPEN";
        positionArm(arm_movement);
    }

    void arm_dropBallClose() {
        ROS_INFO("Mother Brain: Moving arm to drop ball close.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "DROP_BALL_CLOSE";
        positionArm(arm_movement);
    }

    void arm_search() {
        ROS_INFO("Mother Brain: Moving arm to search mode.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "SEARCH";
        positionArm(arm_movement);
    }

    void positionArm(coconuts_common::ArmMovement arm_movement) {
        ROS_INFO("Mother Brain: Sending Command to Arm");
        motor_control_pub.publish(arm_movement);
    }


    ~mother_brain() { }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mother_brain");
    mother_brain mother_brain_h;


    ROS_INFO("Mother Brain Started");

    if (ros::ok()) {
        mother_brain_h.arm_search();
    }

    while(ros::ok()) {
	
        // This structure would repeat in each callback
        // Logic that always runs
        // CASE statement specific logic
        // Explicit debugging 

        if (prev_behavior_state_ != behavior_state_) {
            ROS_INFO("State changed from [%d] to [%d].", prev_behavior_state_, behavior_state_);
            prev_behavior_state_ = behavior_state_;
            if ( behavior_state_ == INIT) {
                mother_brain_h.arm_search();
            }
        }

        switch ( behavior_state_ ) {

            case INIT:
                break;

            case MANUAL:
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

                //mother_brain_h.find_ball();

                break;

            case MOVE_TO_BALL:
                mother_brain_h.move_to_ball(); 
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
