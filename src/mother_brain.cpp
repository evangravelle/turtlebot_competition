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

/*
 * TODO
 *
 * Ensure we're not using DEFAULT states for anything. Everything sholud have a home...
 *
 */

int behavior_state_;
int prev_behavior_state_;
int behavior_sub_state_;
int prev_behavior_sub_state_;

class mother_brain {

private:

    ros::NodeHandle nodeh;

	// Topics we subscribe to
    ros::Subscriber 
        control_override_sub,
        control_substate_sub,
        detect_ball_forward_sub,
        detect_goal_forward_sub;

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
        detect_goal_forward_sub  = nodeh.subscribe<geometry_msgs::Point>("/detect_goal_forward/ball_pixel", 1, &mother_brain::find_goal_callback, this);

        // Publishers
        control_state_pub = nodeh.advertise<coconuts_common::ControlState>("/control_state", 10);
        goal_pose_pub = nodeh.advertise<geometry_msgs::TransformStamped>("/goal_pose", 1);
        cmd_vel_pub = nodeh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
        motor_control_pub = nodeh.advertise<coconuts_common::ArmMovement>("/motor_control", 1);


    }


    void control_override_receive(const coconuts_common::ControlState::ConstPtr& control_msg) {

        behavior_state_ = control_msg->state;
        behavior_sub_state_ = control_msg->sub_state;
        ROS_INFO("Mother Brain: Forced control state to [%d], [%d].", behavior_state_, behavior_sub_state_);

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

    // TODO: Failure Condition
    void drop_ball() {

        if (behavior_state_ == DROP_BALL) {

            switch(behavior_sub_state_) {
                case BALL_DROPPED:
                    ROS_INFO("Mother Brain (DROP_BALL): BALL_DROPPED, going to START.");
                    behavior_state_ = START;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                case DROP_BALL_FAILED:
                    ROS_INFO("Mother Brain (DROP_BALL): DROP_BALL_FAILED, going to FIND_BALL.");
                    behavior_state_ = FIND_BALL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                default:
                    arm_drop_ball_open();
                    ros::Duration(3.0).sleep();
                    behavior_sub_state_ = BALL_DROPPED;
                    break;
            }
        }

    }

    void move_to_goal() {

        if (behavior_state_ == MOVE_TO_GOAL) {

            switch(behavior_sub_state_) {
                case MOVING_TO_GOAL:
                    //ROS_INFO("Mother Brain (MOVE_TO_GOAL): Moving To Ball.");
                    break;

                case AT_GOAL:
                    ROS_INFO("Mother Brain (MOVE_TO_GOAL): AT_GOAL, going to DROP_BALL.");
                    behavior_state_ = DROP_BALL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                case MOVE_TO_GOAL_FAILED:
                    ROS_INFO("Mother Brain (MOVE_TO_GOAL): MOVE_TO_GOAL_FAILED, going to FIND_GOAL.");
                    behavior_state_ = FIND_GOAL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                default:
                    ROS_INFO("Mother Brain (MOVE_TO_GOAL): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }

        }
    }

    void find_goal() {

        if (behavior_state_ == FIND_GOAL) {

            switch(behavior_sub_state_){
                case SEARCH_FOR_GOAL:
                    //ROS_INFO("Mother Brain (FIND_GOAL): Searching for Goal.");
                    break;

                case GOAL_FOUND:
                    ROS_INFO("Mother Brain (FIND_GOAL): GOAL_FOUND, going to MOVE_TO_GOAL.");
                    behavior_state_ = MOVE_TO_GOAL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                default:
                    ROS_INFO("Mother Brain (FIND_GOAL): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }
        }

    }

    void find_goal_callback(const geometry_msgs::Point::ConstPtr& msg) {

        if (behavior_state_ == FIND_GOAL) {

                ROS_INFO("Mother Brain (FIND_GOAL): Callback Executed, going to MOVE_TO_GOAL.");
                behavior_state_ = MOVE_TO_GOAL;
                behavior_sub_state_ = MOVING_TO_GOAL;

        }
    }

    void pick_up_ball() {

        if (behavior_state_ == PICK_UP_BALL) {

            switch(behavior_sub_state_) {
                case ATTEMPT_PICK_UP:
                    // grab ball
                    arm_grab_ball_close();
                    ros::Duration(3.0).sleep();
                    // move to validate posltion
                    arm_check();
                    ros::Duration(3.0).sleep();
                    behavior_sub_state_ = CHECK_BALL;
                    break;

                case CHECK_BALL:
                    ROS_INFO("Mother Brain (PICK_UP_BALL):  CHECK_BALL, still checking");
                    break;

                case GOT_BALL:
                    ROS_INFO("Mother Brain (PICK_UP_BALL): GOT_BALL, going to FIND_GOAL");
                    behavior_state_ = FIND_GOAL;
                    behavior_sub_state_ = SEARCH_FOR_GOAL;
                    break;

                case GOT_BALL_FAILED:
                    ROS_INFO("Mother Brain (PICK_UP_BALL): GOT_BALL_FAILED, going to FIND_BALL.");
                    behavior_state_ = FIND_BALL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                default:
                    ROS_INFO("Mother Brain (PICK_UP_BALL): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }
        }
    }

    void start() {

        if (behavior_state_ == START) {
            ROS_INFO("Mother Brain (START): Starting Initialization.");
            arm_search();
            ros::Duration(3.0).sleep();
            ROS_INFO("Mother Brain (START): Finished Initialization, moving to FIND_BALL.");
            behavior_state_ = FIND_BALL;
            behavior_sub_state_ = DEFAULT_SUB_STATE;
        }

    }

    void end() {

        if (behavior_state_ == END) {
            ROS_INFO("Mother Brain (END): Shutting Down.");
            arm_search();
            ros::Duration(3.0).sleep();
            ROS_INFO("Mother Brain (END): Going to INIT.");
            behavior_state_ = INIT;
            behavior_sub_state_ = DEFAULT_SUB_STATE;
        }
    }

    void config() {

        if (behavior_state_ == CONFIG) {

            switch(behavior_sub_state_){
                case CONFIG_GOAL_COLOR:
                    //ROS_INFO("Mother Brain (CONFIG): Setting Goal Color.");
                    break;

                case CONFIG_CEILING:
                    //ROS_INFO("Mother Brain (CONFIG): Setting Ceiling.");
                    break;

                case CONFIG_BALL_COLOR:
                    //ROS_INFO("Mother Brain (CONFIG): Setting Ball Color.");
                    break;

                default:
                    ROS_INFO("Mother Brain (CONFIG): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }
        }
    }

    // TODO: Failure Recovery
    void move_to_ball() {

        if (behavior_state_ == MOVE_TO_BALL) {

            switch(behavior_sub_state_) {
                case MOVING_TO_BALL:
                    //ROS_INFO("Mother Brain (MOVE_TO_BALL): MOVING_TO_BALL.");
                    break;

                case AT_BALL:
                    ROS_INFO("Mother Brain (MOVE_TO_BALL): AT_BALL, going to attempt pick up.");
                    behavior_state_ = PICK_UP_BALL;
                    behavior_sub_state_ = ATTEMPT_PICK_UP;
                    break;

                case CENTER_ON_BALL:
                    ROS_INFO("Mother Brain (MOVE_TO_BALL): CENTER_ON_BALL, centering.");
                    break;

                case MOVE_TO_BALL_FAILED:
                    ROS_INFO("Mother Brain (MOVE_TO_BALL): MOVE_TO_BALL_FAILED, going to FIND_BALL.");
                    behavior_state_ = FIND_BALL;
                    behavior_sub_state_ = DEFAULT_SUB_STATE;
                    break;

                default:
                    ROS_INFO("Mother Brain (MOVE_TO_BALL): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }
        }

    }

    void find_ball_callback(const geometry_msgs::Point::ConstPtr& msg) {

        if (behavior_state_ == FIND_BALL) {
                ROS_INFO("Mother Brain (FIND_BALL): Callback Executed, going to MOVE_TO_BALL.");
                behavior_state_ = MOVE_TO_BALL;
                behavior_sub_state_ = MOVING_TO_BALL;
        }

    }

    void find_ball() {

        if (behavior_state_ == FIND_BALL) {

            switch(behavior_sub_state_) {

                case DEFAULT_SUB_STATE:
                    ROS_INFO("Mother Brain (FIND_BALL): DEFAULT_SUB_STATE, starting search.");
                    behavior_sub_state_ = SEARCH_FOR_BALL;
                    break;

                case SEARCH_FOR_BALL:
                    //ROS_INFO("Mother Brain (FIND_BALL): SEARCH_FOR_BALL contines....");
                    break;

                case BALL_FOUND:
                    ROS_INFO("Mother Brain (FIND_BALL): BALL_FOUND, going to MOVE_TO_BALL.");
                    behavior_state_ = MOVE_TO_BALL;
                    behavior_sub_state_ = MOVING_TO_BALL;
                    break;

                default:
                    ROS_INFO("Mother Brain (FIND_BALL): DEFAULT called [%d].", behavior_sub_state_);
                    break;
            }

            /*
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
            */
        }
    }
        

    void publish_goal(tf::StampedTransform transform) {
        // How do we confert the StampedTransform to a geometry_msgs::PoseStamped
        geometry_msgs::TransformStamped msg;
        tf::transformStampedTFToMsg(transform, msg);
        goal_pose_pub.publish(msg);
    }

    void arm_grab_ball_open() {
        ROS_INFO("Mother Brain: Moving arm to grab ball open.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "GRAB_BALL_OPEN";
        positionArm(arm_movement);
    }

    void arm_grab_ball_close() {
        ROS_INFO("Mother Brain: Moving arm to grab ball close.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "GRAB_BALL_CLOSE";
        positionArm(arm_movement);
    }

    void arm_drop_ball_open() {
        ROS_INFO("Mother Brain: Moving arm to drop ball open.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "DROP_BALL_OPEN";
        positionArm(arm_movement);
    }

    void arm_drop_ball_close() {
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

    void arm_check() {
        ROS_INFO("Mother Brain: Moving arm to check.");
        coconuts_common::ArmMovement arm_movement;
        arm_movement.type = "POSE";
        arm_movement.pose = "CHECK_BALL";
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


    ROS_INFO("Mother Brain (main): Started");

    while(ros::ok()) {
	
        // This structure would repeat in each callback
        // Logic that always runs
        // CASE statement specific logic
        // Explicit debugging 

        if (prev_behavior_sub_state_ != behavior_sub_state_) {
            prev_behavior_sub_state_ = behavior_sub_state_;
        }

        if (prev_behavior_state_ != behavior_state_) {
            ROS_INFO("State changed from [%d] to [%d].", prev_behavior_state_, behavior_state_);
            prev_behavior_state_ = behavior_state_;
        }

        switch ( behavior_state_ ) {

            case INIT:
                break;

            case MANUAL:
                break;

            case START:
                mother_brain_h.start();
                break;

            case END:
                mother_brain_h.end();
                break;

            case CONFIG:
                mother_brain_h.config();
                break;

            case FIND_GOAL:
                mother_brain_h.find_goal();
                break;

            case MOVE_TO_GOAL:
                mother_brain_h.move_to_goal();
                break;

            case FIND_BALL:
                mother_brain_h.find_ball();
                break;

            case MOVE_TO_BALL:
                mother_brain_h.move_to_ball(); 
                break;

            case PICK_UP_BALL:
                mother_brain_h.pick_up_ball();
                break;

            case DROP_BALL:
                mother_brain_h.drop_ball();
                break;

            default: 
                ROS_ERROR("Default called on case stament.  Unexpected state [%d] in mother_brain:main().", behavior_state_);

        }

        // Publish Current state and sub_state
        //
        mother_brain_h.publish_state();

        ros::spinOnce();
    }

    ROS_INFO("Mother Brain (main): o/\" Yo holmes smell ya later \"\\o");
}
