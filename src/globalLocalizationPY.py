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

# loop over the images to find the template in
for imagePath in np.linspace(0, 1,  1)[::-1]:


	# load the image, convert it to grayscale, and initialize the
	# bookkeeping variable to keep track of the matched region
	#image = cv2.imread(imagePath)
	image = cv2.imread("/home/aaron/ceiling/t.png")
	postingImage = np.copy(image)



	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	(rows,cols) = template.shape[:2]

class GL:


	def __init__(self):
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('/pose', Pose, queue_size=1)
		self.br= tf2_ros.TransformBroadcaster()
#		self.tfPub = rospy.Publisher('/tf', TransformStamped, queue_size=1)
		self.sub =rospy.Subscriber('/camera_up/image_raw', Image, self.callback)
		self.sub2 =rospy.Subscriber('/odom',Odometry , self.odometryCB)
		self.sub3 =rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.twistCB)
		self.pose = Pose()
		self.pose.position.x=0
		self.pose.position.y=0
		self.sx=0
		self.sy=0
		self.angle=0
		self.turning=0
		self.signal=Twist()
		self.init=False

		self.lastOdomX=0
		self.lastOdomY=0
		self.lastQuaternion = type('Quaternion', (), {})()
		self.lastQuaternion.__class__.__bases__
		self.lastYaw=0
		self.t= TransformStamped()
		self.q=[0,0,0,0]


	def odometryCB(self,data):
##UPDATE POSE MESSAGES
		if self.lastOdomX is 0 and self.lastOdomY is 0 and self.lastYaw is 0:
			self.lastOdomX = data.pose.pose.position.x
			self.lastOdomY = data.pose.pose.position.y
			self.lastQuaternion.z=data.pose.pose.orientation.z
			self.lastQuaternion.w=data.pose.pose.orientation.w
			(roll,pitch,yaw) = euler_from_quaternion([0,0,self.lastQuaternion.z,self.lastQuaternion.w])
			self.lastYaw = yaw
		else:
			self.pose.position.x+=math.sin(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.pose.position.y+=math.cos(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.lastOdomX = data.pose.pose.position.x
			self.lastOdomY = data.pose.pose.position.y
			self.lastQuaternion.z=data.pose.pose.orientation.z
			self.lastQuaternion.w=data.pose.pose.orientation.w
			(roll,pitch,yaw) = euler_from_quaternion([0,0,self.lastQuaternion.z,self.lastQuaternion.w])
			self.angle+=57.2958*(yaw-self.lastYaw)
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
		self.pub.publish(self.pose)
		
		(startX, startY) = (int(self.pose.position.x), int(self.pose.position.y))
		(endX, endY) = (int((self.pose.position.x+40*math.sin(self.angle*0.0174533)) ), int((self.pose.position.y+40*math.cos(self.angle*0.0174533)) ))

		postingImage = np.copy(image)
		resized=postingImage

		cv2.line(resized, (startX, startY), (endX, endY), (0, 255, 0), 2)
		cv2.circle(resized,(startX,startY), 40, (0,255,0), 5)
		cv2.imshow("Image", postingImage)
		cv2.waitKey(1)










	def twistCB(self,data):
		if data.angular.z is not 0:
			self.turning = time.time()
		self.signal.angular=data.angular
		self.signal.linear=data.linear





	def callback(self,data):
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
		resized=postingImage

		if self.pose.position.x is not 0 and self.pose.position.y is not 0 and self.pose.position.x > 176 and self.pose.position.y> 176:
			resized=resized[(self.pose.position.y*2-350)/2:(self.pose.position.y*2-350)/2+350, (self.pose.position.x*2-350)/2:350+(self.pose.position.x*2-350)/2]



		for scale in np.linspace(self.angle-self.angleRange, self.angle+self.angleRange,  5)[::-1]:
			M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
			template = cv2.warpAffine(tempt,M,(cols,rows))
			template = template[(rows-250)/2:(rows-250)/2+250, (cols-250)/2:250+(cols-250)/2]
			# if the resized image is smaller than the template, then break
			# from the loop

			# detect edges in the resized, grayscale image and apply template
			# matching to find the template in the image
			edged = cv2.Canny(resized, 25, 50)


			result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
			# if we have found a new maximum correlation value, then ipdate
			# the bookkeeping variable
			if found2 is None or maxVal > found2[0]:
				found2 = (maxVal, maxLoc, scale)


		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(maxVal, maxLoc, maxScale) = found2
		(startX, startY) = (int(maxLoc[0]  +125), int(maxLoc[1]+125 ))
		(endX, endY) = (int((maxLoc[0] +125+40*math.sin(maxScale*0.0174533)) ), int((maxLoc[1] +125+40*math.cos(maxScale*0.0174533)) ))

		# draw a bounding box around the detected result and display the image


		cv2.line(resized, (startX, startY), (endX, endY), (0, 0, 255), 2)
		cv2.circle(resized,(startX,startY), 40, (0,0,255), 5)


		if self.pose.position.x is not 0 and self.pose.position.y is not 0 and self.pose.position.x > 176 and self.pose.position.y> 176:
			#image=image[(self.y*2-350)/2:(self.y*2-350)/2+350, (self.x*2-350)/2:350+(self.x*2-350)/2]
			self.pose.position.x+=-175+startX#-175+startX
			self.pose.position.y+=-175+startY#175+startY
		else:
			self.pose.position.x=startX#-175+startX
			self.pose.position.y=startY#175+startY
		self.angle=maxScale





		cv2.imshow("Image", postingImage)
		cv2.imshow("Image2", template)
		cv2.imshow("Image3", edged)
		cv2.waitKey(1)

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
		self.pub.publish(self.pose)


	def kalman(self):
		while True and self.init is True:
			self.pose.position.x+=math.sin(self.angle*0.0174533)*self.signal.linear.x*4.
			self.pose.position.y+=math.cos(self.angle*0.0174533)*self.signal.linear.x*4.
			self.angle+=self.signal.angular.z

			postingImage = np.copy(image)
			resized=postingImage

			(startX, startY) = (int(self.pose.position.x), int(self.pose.position.y))
			(endX, endY) = (int((self.pose.position.x+40*math.sin(self.angle*0.0174533)) ), int((self.pose.position.y+40*math.cos(self.angle*0.0174533)) ))

			cv2.line(resized, (startX, startY), (endX, endY), (255, 0, 0), 2)
			cv2.circle(resized,(startX,startY), 40, (255,0,0), 5)
			cv2.imshow("Image", postingImage)
			cv2.waitKey(10)




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



