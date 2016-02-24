#!/usr/bin/env python
# USAGE
# python match.py --template cod_logo.png --images images

# import the necessary packages
import numpy as np
import rospy
from std_msgs.msg import String
import time
import math
import sys
import glob
import cv2

import tf
import tf2_ros

from cv2 import __version__
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped


# load the image image, convert it to grayscale, and detect edges
#template = cv2.imread(args["template"])
template = cv2.imread("/home/aaron/ceiling/p10.png")
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 10, 100)
tempt=template
(tH, tW) = template.shape[:2]
#cv2.imshow("Template", template)

def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 0.
    else:
        return x

image = cv2.imread("/home/aaron/ceiling/t.png")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#image = cv2.Canny(image, 25, 50)
imageCanny=cv2.Canny(image, 25, 50)
postingImage = np.copy(image)
(rows,cols) = template.shape[:2]

class GL:


	def __init__(self):
		self.bridge = CvBridge()
#		self.pub = rospy.Publisher('/pose', Pose, queue_size=1)
		self.pubImage=rospy.Publisher('/localization/Image', Image, queue_size=1)
		self.br= tf2_ros.TransformBroadcaster()
#		self.tfPub = rospy.Publisher('/tf', TransformStamped, queue_size=1)
		self.sub =rospy.Subscriber('/camera_up/image_raw', Image, self.callback)
		self.sub2 =rospy.Subscriber('/odom',Odometry , self.odometryCB)
		self.sub3 =rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.twistCB)
		self.pose = Pose()
		self.pose.position.x=550
		self.pose.position.y=550
		self.sx=0
		self.sy=0
		self.angle=0
		self.turning=0
		self.signal=Twist()
		self.init=True
		self.kalmanRun=False

		self.ceilingFlag=False
		self.ceilingMeasurement = Pose()
		self.ceilingMeasurement.position.x=550
		self.ceilingMeasurement.position.y=550
		self.ceilingAngle=0
		self.ceilingConfidence=0

		self.odomFlag=False
		self.odomMeasurement = Pose()
		self.odomMeasurement.position.x=550
		self.odomMeasurement.position.y=550
		self.odomAngle=0

		self.lastPoseX=0
		self.lastPoseY=0
		self.lastPoseAngle=0;
		self.lastOdomX=0
		self.lastOdomY=0
		self.lastQuaternion = type('Quaternion', (), {})()
		self.lastQuaternion.__class__.__bases__
		self.lastYaw=0
		self.t= TransformStamped()
		self.q=[0,0,0,0]


	def odometryCB(self,data):
##UPDATE POSE MESSAGES
		self.odomFlag=True
		if self.lastOdomX is 0 and self.lastOdomY is 0 and self.lastYaw is 0:
			self.lastPoseX=self.pose.position.x
			self.lastPoseY=self.pose.position.y
			self.lastPoseAngle=self.angle
			self.lastOdomX = data.pose.pose.position.x
			self.lastOdomY = data.pose.pose.position.y
			self.lastQuaternion.z=data.pose.pose.orientation.z
			self.lastQuaternion.w=data.pose.pose.orientation.w
			(roll,pitch,yaw) = euler_from_quaternion([0,0,self.lastQuaternion.z,self.lastQuaternion.w])
			self.lastYaw = yaw
		else:
			self.pose.position.x+=sign(data.twist.twist.linear.x)*math.sin(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.pose.position.y+=sign(data.twist.twist.linear.x)*math.cos(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.odomMeasurement.position.x=self.lastPoseX+sign(data.twist.twist.linear.x)*math.sin(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.odomMeasurement.position.y=self.lastPoseY+sign(data.twist.twist.linear.x)*math.cos(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.lastPoseX=self.pose.position.x
			self.lastPoseY=self.pose.position.y
			self.lastOdomX = data.pose.pose.position.x
			self.lastOdomY = data.pose.pose.position.y
			self.lastQuaternion.z=data.pose.pose.orientation.z
			self.lastQuaternion.w=data.pose.pose.orientation.w
			(roll,pitch,yaw) = euler_from_quaternion([0,0,self.lastQuaternion.z,self.lastQuaternion.w])
			self.odomAngle=self.lastPoseAngle+57.2958*(yaw-self.lastYaw)
			self.lastPoseAngle=self.angle
			self.lastYaw = yaw



##UPDATE TRANSFORM MESSAGES
		self.t.header.stamp= rospy.Time.now()
		self.t.header.frame_id = "map";
		self.t.child_frame_id = "base_footprint";
		self.t.transform.translation.x = self.pose.position.x/250.;
		self.t.transform.translation.y = self.pose.position.y/250.;
		self.q= tf.transformations.quaternion_from_euler(0, 0, self.angle*0.0174533)
		self.t.transform.rotation.x = self.q[0]
		self.t.transform.rotation.y = self.q[1]
		self.t.transform.rotation.z = self.q[2]
		self.t.transform.rotation.w = self.q[3]
		self.br.sendTransform(self.t)
		
#		(startX, startY) = (int(self.pose.position.x), int(self.pose.position.y))
#		(endX, endY) = (int((self.pose.position.x+40*math.sin(self.angle*0.0174533)) ), int((self.pose.position.y+40*math.cos(self.angle*0.0174533)) ))

#		postingImage = np.copy(image)
#		resized=postingImage

#		cv2.line(resized, (startX, startY), (endX, endY), (0, 255, 0), 2)
#		cv2.circle(resized,(startX,startY), 40, (0,255,0), 5)
#		cv2.imshow("Image", postingImage)
#		cv2.waitKey(1)



	def twistCB(self,data):
		if data.angular.z is not 0:
			self.turning = time.time()
		self.signal.angular=data.angular
		self.signal.linear=data.linear





	def callback(self,data):
		self.ceilingFlag=True
		self.init=True
		self.start = time.time()
		if time.time()-self.turning > 2:
			self.angleRange=12
		else:
			self.angleRange=35



		template= self.bridge.imgmsg_to_cv2(data, "bgr8")
		(rows,cols) = template.shape[:2]

		cv2.waitKey(1)
		template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		template = cv2.Canny(template, 25, 50)
		tempt=template

		found2 = None
		postingImage = np.copy(image)
		resized=np.copy(imageCanny)

		if self.pose.position.x is not 0 and self.pose.position.y is not 0 and self.pose.position.x > 176 and self.pose.position.y> 176:
			resized=resized[(self.pose.position.y*2-350)/2:(self.pose.position.y*2-350)/2+350, (self.pose.position.x*2-350)/2:350+(self.pose.position.x*2-350)/2]



		for scale in np.linspace(self.angle-self.angleRange, self.angle+self.angleRange,  5)[::-1]:
			M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
			template = cv2.warpAffine(tempt,M,(cols,rows))
			template = template[(rows-250)/2:(rows-250)/2+250, (cols-250)/2:250+(cols-250)/2]
			edged=resized
			result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
			if found2 is None or maxVal > found2[0]:
				found2 = (maxVal, maxLoc, scale)

		(maxVal, maxLoc, maxScale) = found2
		(startX, startY) = (int(maxLoc[0]  +125), int(maxLoc[1]+125 ))
		(endX, endY) = (int((maxLoc[0] +125+40*math.sin(maxScale*0.0174533)) ), int((maxLoc[1] +125+40*math.cos(maxScale*0.0174533)) ))
		if self.pose.position.x is not 550 and self.pose.position.y is not 550 and self.pose.position.x > 176 and self.pose.position.y> 176:
			self.ceilingMeasurement.position.x=self.pose.position.x-175+startX
			self.ceilingMeasurement.position.y=self.pose.position.y-175+startY
		else:
			self.pose.position.x=startX
			self.pose.position.y=startY
		self.ceilingAngle=maxScale

#		cv2.imshow("Image2", template)
#		cv2.imshow("Image3", edged)
#		cv2.waitKey(1)

		self.t.header.stamp= rospy.Time.now()
		self.t.header.frame_id = "map";
		self.t.child_frame_id = "base_footprint";
		self.t.transform.translation.x = self.pose.position.x/250.0;
		self.t.transform.translation.y = self.pose.position.y/250.0;
		self.q= tf.transformations.quaternion_from_euler(0, 0, (self.angle+90)*0.0174533)
		self.t.transform.rotation.x = self.q[0]
		self.t.transform.rotation.y = self.q[1]
		self.t.transform.rotation.z = self.q[2]
		self.t.transform.rotation.w = self.q[3]
		self.br.sendTransform(self.t)
		self.ceilingConfidence=maxVal/60000000.

		if self.ceilingConfidence>1:
			self.ceilingConfidence=1.0



	def kalman(self):
		while not rospy.is_shutdown():
#			self.pose.position.x+=math.sin(self.angle*0.0174533)*self.signal.linear.x*4.
#			self.pose.position.y+=math.cos(self.angle*0.0174533)*self.signal.linear.x*4.
#			self.angle+=self.signal.angular.z

			if self.ceilingFlag is True:
				self.pose.position.x=self.pose.position.x+self.ceilingConfidence*(-self.pose.position.x+self.ceilingMeasurement.position.x)
				self.pose.position.y=self.pose.position.y+self.ceilingConfidence*(-self.pose.position.y+self.ceilingMeasurement.position.y)
				self.angle=self.angle+.4*(-self.angle+self.ceilingAngle)
				print str(self.ceilingConfidence)
				self.ceilingFlag=False

#			if self.odomFlag is True:
#				print str(self.pose.position.x-self.odomMeasurement.position.x)
#				self.pose.position.x=self.pose.position.x+.8*(-self.pose.position.x+self.odomMeasurement.position.x)
#				self.pose.position.y=self.pose.position.y+.8*(-self.pose.position.y+self.odomMeasurement.position.y)
#				self.angle=self.angle+.5*(-self.angle+self.odomAngle)
#				self.lastPoseX=self.pose.position.x
#				self.lastPoseY=self.pose.position.y
#				self.lastPoseAngle=self.angle
#				self.odomFlag=False


			postingImage = np.copy(image)
			resized=postingImage

			(startX, startY) = (int(self.pose.position.x), int(self.pose.position.y))
			(endX, endY) = (int((self.pose.position.x+40*math.sin(self.angle*0.0174533)) ), int((self.pose.position.y+40*math.cos(self.angle*0.0174533)) ))
			cv2.line(resized, (startX, startY), (endX, endY), (255, 255, 0), 2)
			cv2.circle(resized,(startX,startY), 40, (255,255,0), 5)

			(startX, startY) = (int(self.odomMeasurement.position.x), int(self.odomMeasurement.position.y))
			(endX, endY) = (int((self.odomMeasurement.position.x+40*math.sin(self.odomAngle*0.0174533)) ), int((self.odomMeasurement.position.y+40*math.cos(self.odomAngle*0.0174533)) ))
			cv2.line(resized, (startX, startY), (endX, endY), (0, 255, 0), 2)
			cv2.circle(resized,(startX,startY), 40, (0,255,0), 5)

			(startX, startY) = (int(self.ceilingMeasurement.position.x), int(self.ceilingMeasurement.position.y))
			(endX, endY) = (int((self.ceilingMeasurement.position.x+40*math.sin(self.ceilingAngle*0.0174533)) ), int((self.ceilingMeasurement.position.y+40*math.cos(self.ceilingAngle*0.0174533)) ))
			cv2.line(resized, (startX, startY), (endX, endY), (0, 255, 255), 2)
			cv2.circle(resized,(startX,startY), 40, (0,255,255), 5)


			self.pubImage.publish(self.bridge.cv2_to_imgmsg(postingImage, "bgr8"))
#			cv2.imshow("Image", postingImage)
			cv2.waitKey(10)
#			self.pub.publish(self.pose)




def main(args):
	rospy.init_node('image_converter', anonymous=True)
	gl = GL()
	gl.kalman()
	try:

		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

