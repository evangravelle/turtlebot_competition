#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <states.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/SensorStatus.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/String.h>
#include <cmath>

// Initialize variables
geometry_msgs::Pose waypoint, goal;
ros::Time current_time;
coconuts_common::ControlState current_state;
bool left_obstacle, right_obstacle;

int left_detect_counter=0, right_detect_counter=0;
int MAX_DETECT_COUNT = 4;
int COUNT_DETECT_THRESH = 3;

double inches_to_meters = 0.0254;
double obstacle_distance = .5;
geometry_msgs::TransformStamped odom;
double t;

void stateCallback(const coconuts_common::ControlState::ConstPtr& control_msg) {
    current_state.state = control_msg->state;
    current_state.sub_state = control_msg->sub_state;
}

void sensorCallback(const coconuts_common::SensorStatus::ConstPtr& sensor_msg) {

    for (int i = 0; i < sensor_msg->sensor_readings.size(); i++ ) {
        ROS_INFO("Explorer: Looking at sensor [%d] reading [%d].", sensor_msg->sensor_readings[i].sensor, sensor_msg->sensor_readings[i].reading);

        switch (sensor_msg->sensor_readings[i].sensor) {

            case 1:
                if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 45) {
                    right_detect_counter++;
                } 
                else {
                    right_detect_counter--;
                }

                right_detect_counter = std::max(std::min(right_detect_counter, MAX_DETECT_COUNT), 0);
                right_obstacle = right_detect_counter >= COUNT_DETECT_THRESH;
                break;

            case 2:
                if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 45 {
                    left_detect_counter++;
                } 
                else {
                    left_detect_counter--;
                }
                left_detect_counter = std::max(std::min(left_detect_counter, MAX_DETECT_COUNT), 0);
                left_obstacle = left_detect_counter >= COUNT_DETECT_THRESH;
                break;
            
            default:
                break;
        }
    }
}

void goalCallback(const geometry_msgs::Pose::ConstPtr& goal_msg) {
        goal.position.x = goal_msg->position.x;
        goal.position.y = goal_msg->position.y;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "waypoints");
	ros::NodeHandle nh;

    
    //Status messages, more readable than waypoints_pub
    ros::Publisher status_string_pub = nh.advertise<std_msgs::String>("/wtf_am_i_doing", 1);

    ros::Publisher waypoints_pub = nh.advertise<geometry_msgs::Pose>("/waypoint", 1, true);
    ros::Subscriber control_state_sub = nh.subscribe<coconuts_common::ControlState>("/control_state", 1, stateCallback);
    ros::Subscriber sensor_sub  = nh.subscribe<coconuts_common::SensorStatus>("/sensor_status", 1, sensorCallback);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Pose>("/goal", 1, goalCallback);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    goal.position.x = -1;
    goal.position.y = -1;
    double t = 0;

    std::stringstream ss;
    std_msgs::String wtf_status_msg;
    

    ros::Rate rate(5.0);
    while(ros::ok()) {
        ss.str("");
        tf::StampedTransform transform;
        try{
            odom = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
        }
    	ROS_INFO("Transform found!");
        if(current_state.sub_state==SEARCH_FOR_GOAL || current_state.sub_state==SEARCH_FOR_BALL || 
            current_state.sub_state==MOVING_TO_ORANGE || current_state.sub_state==MOVING_TO_GREEN || current_state.sub_state == MOVING_TO_GOAL ){

        // Wait 10 seconds before considering publishing a new waypoint
            if (right_obstacle && goal.position.x > 0 && 
                (t < 0.0001 || ros::Time::now().toSec() - t > 5)) {
               	ROS_INFO("right sensor blocked");
                float relative_x = goal.position.x - odom.transform.translation.x;
                float relative_y = goal.position.y - odom.transform.translation.y;
                double angle = atan2(relative_y, relative_x);

                waypoint.position.x = odom.transform.translation.x + obstacle_distance*cos(angle - M_PI/3.0);
                waypoint.position.y = odom.transform.translation.y + obstacle_distance*sin(angle - M_PI/3.0);
                waypoints_pub.publish(waypoint);
                t = ros::Time::now().toSec();

                
                ss << "Right sensor blocked, rerouting to " << waypoint.position.x << "," << waypoint.position.y;
                wtf_status_msg.data = ss.str();
                status_string_pub.publish(wtf_status_msg);

                left_detect_counter=0;
                right_detect_counter=0;
                left_obstacle=false;
                right_obstacle=false;
            }

            // Wait 10 seconds before considering publishing a new waypoint
            else if (left_obstacle && goal.position.x > 0 && 
                (t < 0.0001 || ros::Time::now().toSec() - t > 5)) {
                ROS_INFO("left sensor blocked");
                float relative_x = goal.position.x - odom.transform.translation.x;
                float relative_y = goal.position.y - odom.transform.translation.y;
                double angle = atan2(relative_y, relative_x);

                waypoint.position.x = odom.transform.translation.x + obstacle_distance*cos(angle + M_PI/3.0);
                waypoint.position.y = odom.transform.translation.y + obstacle_distance*sin(angle + M_PI/3.0);
                waypoints_pub.publish(waypoint);
                t = ros::Time::now().toSec();
                
                ss << "Left sensor blocked, rerouting to " << waypoint.position.x << "," << waypoint.position.y;
                wtf_status_msg.data = ss.str();
                status_string_pub.publish(wtf_status_msg);

                left_detect_counter=0;
                right_detect_counter=0;
                left_obstacle=false;
                right_obstacle=false;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
