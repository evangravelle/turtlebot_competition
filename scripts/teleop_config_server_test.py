#!/usr/bin/env python

import roslib
roslib.load_manifest('coconuts_master')
import rospy
import actionlib

from coconuts_master.msg import ConfigStereoCameraAction, ConfigStereoCameraGoal

class ConfigStereoCameraServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('config_stereo_camera', ConfigStereoCameraAction, self.execute, False)
		print "Setting up server"
		self.server.start()

	def execute(self, goal):
		print "Executing Goal"
		self.server.set_succeeded()


if __name__ == '__main__':
	rospy.init_node('teleop_config_test_server');
	server = ConfigStereoCameraServer()
	rospy.spin()
