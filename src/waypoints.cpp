#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <states.h>
#include <coconuts_common/ControlState.h>
#include <coconuts_common/SensorStatus.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

// Initialize variables
geometry_msgs::Pose waypoint, goal;
ros::Time current_time;
coconuts_common::ControlState current_state;
bool left_obstacle, right_obstacle;
int left_counter, right_counter;
double inches_to_meters = 0.0254;
double obstacle_distance = .75;
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
                if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 50) {
                    right_obstacle = true;
                    right_counter++;
                } 
                else {
                    right_obstacle = false;
                    right_counter = 0;
                }
                break;

            case 2:
                if (sensor_msg->sensor_readings[i].reading > 0 && sensor_msg->sensor_readings[i].reading < 50) {
                    left_obstacle = true;
                    left_counter++;
                } 
                else {
                    left_obstacle = false;
                    left_counter = 0;
                }
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

    ros::Publisher waypoints_pub = nh.advertise<geometry_msgs::Pose>("/waypoint", 1, true);
    ros::Subscriber control_state_sub = nh.subscribe<coconuts_common::ControlState>("/control_state", 1, stateCallback);
    ros::Subscriber sensor_sub  = nh.subscribe<coconuts_common::SensorStatus>("/sensor_status", 1, sensorCallback);
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::Pose>("/goal", 1, goalCallback);
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    goal.position.x = -1;
    goal.position.y = -1;
    double t = 0;

    ros::Rate rate(5.0);
    while(ros::ok()) {
        tf::StampedTransform transform;
        try{
            odom = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
        }
	ROS_INFO("Transform found!");
        // Wait 10 seconds before considering publishing a new waypoint
        if (current_state.sub_state == MOVING_TO_GOAL && right_counter > 3 && goal.position.x > 0 && 
            (t < 0.0001 || ros::Time::now().toSec() - t > 10)) {
       	ROS_INFO("right sensor blocked");
            float relative_x = goal.position.x - odom.transform.translation.x;
            float relative_y = goal.position.y - odom.transform.translation.y;
            double angle = atan2(relative_y, relative_x);

            waypoint.position.x = odom.transform.translation.x + obstacle_distance*cos(angle - M_PI/3.0);
            waypoint.position.y = odom.transform.translation.y + obstacle_distance*sin(angle - M_PI/3.0);
            waypoints_pub.publish(waypoint);
            t = ros::Time::now().toSec();
        }

        // Wait 10 seconds before considering publishing a new waypoint
        else if (current_state.sub_state == MOVING_TO_GOAL && left_counter > 3 && goal.position.x > 0 && 
            (t < 0.0001 || ros::Time::now().toSec() - t > 10)) {
            ROS_INFO("left sensor blocked");
            float relative_x = goal.position.x - odom.transform.translation.x;
            float relative_y = goal.position.y - odom.transform.translation.y;
            double angle = atan2(relative_y, relative_x);

            waypoint.position.x = odom.transform.translation.x + obstacle_distance*cos(angle + M_PI/3.0);
            waypoint.position.y = odom.transform.translation.y + obstacle_distance*sin(angle + M_PI/3.0);
            waypoints_pub.publish(waypoint);
            t = ros::Time::now().toSec();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
