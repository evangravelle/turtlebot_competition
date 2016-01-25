#!/usr/bin/env python

import roslib
roslib.load_manifest('coconuts_master')
import rospy
import actionlib

from coconuts_master.msg import ConfigStereoCameraAction, ConfigStereoCameraGoal

if __name__ == '__main__':
	rospy.init_node('teleop_config')
	configStereoClient = actionlib.SimpleActionClient('config_stereo_camera', ConfigStereoCameraAction);
	configStereoClient.wait_for_server()

	goal = ConfigStereoCameraGoal()
	goal.calibration_request = 1
	configStereoClient.send_goal(goal);
	configStereoClient.wait_for_result(rospy.Duration.from_sec(5.0))
