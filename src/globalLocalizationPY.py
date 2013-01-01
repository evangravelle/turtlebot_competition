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
from geometry_msgs.msg import PoseStamped



#To be arguments for starting location and resolution parameters
startingX=600 #550 for 640
startingY=405
bucketX=400
bucketY=400

waypointX=0
waypointY=0

thresholdConfidence=.015
#startingX=130;
#startingY=130;
mapSliceSize=200/2 #prev 350 for 640
sliceSize=175/2 #prev 250

# I dont know if I still need this
template = cv2.imread("/home/ros/catkin_ws/src/coconuts_odroid/src/template_dummy.png")
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 10, 100)
tempt=template
(tH, tW) = template.shape[:2]
size = 200, 200, 3
result2 = np.zeros(size, dtype=np.uint8)

result = np.zeros(size, dtype=np.uint8)
#Returns the sign of a number
def sign(x):
    if x > 0:
        return 1.
    elif x < 0:
        return -1.
    elif x == 0:
        return 0.
    else:
        return x


#Load the ceiling map
image = cv2.imread("/home/ros/catkin_ws/src/coconuts_common/ceiling_1202/ceil_mod.png")
postingImage2 = cv2.imread("/home/ros/catkin_ws/src/coconuts_common/ceiling_1202/ceiling_small.png")
#image = cv2.imread("/home/ros/catkin_ws/src/coconuts_odroid/src/map_small.png")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
posingImage= np.copy(postingImage2)
imageCanny=cv2.Canny(image, 150, 150)
(rows,cols) = template.shape[:2]


#Define a global localization class
class GL:


	def __init__(self):
		self.bridge = CvBridge()
		self.pubImage=rospy.Publisher('/localization/Image', Image, queue_size=1)
		self.br= tf2_ros.TransformBroadcaster()
		self.sub =rospy.Subscriber('/camera_up/image_raw', Image, self.callback)
		self.sub2 =rospy.Subscriber('/odom',Odometry , self.odometryCB)
		self.sub3 =rospy.Subscriber('/mobile_base/commands/velocity', Twist, self.twistCB)
		self.sub4 =rospy.Subscriber('/waypoint', Pose, self.waypointCB)
		self.pose = Pose()
		self.pose.position.x=startingX
		self.pose.position.y=startingY
		self.sx=0
		self.sy=0
		self.angle=0
		self.turning=0
		self.signal=Twist()
		self.init=True
		self.kalmanRun=False
                self.averageTime=[30]*5
                self.averageConfidence=[1]*30
		self.counter=0

#For visualization (Comment out in Final)
		self.poseAndroid=PoseStamped()
		self.pubPoseAndroid=rospy.Publisher('/poseEstimation',PoseStamped,queue_size=1)		

#Associated to ceiling
		self.ceilingFlag=False
		self.ceilingMeasurement = Pose()
		self.ceilingMeasurement.position.x=startingX
		self.ceilingMeasurement.position.y=startingY
		self.ceilingAngle=0
		self.ceilingConfidence=0
		self.lastMeasurement=0

#Associated with Odometry
		self.odomFlag=False
		self.odomMeasurement = Pose()
		self.odomMeasurement.position.x=startingX
		self.odomMeasurement.position.y
		self.odomAngle=0
		self.lastPoseX=0
		self.lastPoseY=0
		self.lastPoseAngle=0;
		self.lastOdomX=0
		self.lastOdomY=0
		self.lastQuaternion = type('Quaternion', (), {})()
		self.lastQuaternion.__class__.__bases__
		self.lastYaw=0

#Transform to be published
		self.t= TransformStamped()
		self.q=[0,0,0,0]


	def waypointCB(self,data):
		waypointX=data.position.x
		waypointY=data.position.y	

#Odometry subscriber and updater
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
			self.pose.position.x+=mapSliceSize/350.*sign(data.twist.twist.linear.x)*math.sin(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
			self.pose.position.y+=mapSliceSize/350.*sign(data.twist.twist.linear.x)*math.cos(self.angle*0.0174533)*250.*math.sqrt((data.pose.pose.position.x-self.lastOdomX)*(data.pose.pose.position.x-self.lastOdomX)+(data.pose.pose.position.y-self.lastOdomY)*(data.pose.pose.position.y-self.lastOdomY))
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
			self.angle-=(57.2958*(yaw-self.lastYaw))
			self.lastPoseAngle=self.angle
			self.lastYaw = yaw




#A simple subsriber to cmd_vel to determine the turtlebot is turning (To determine paramaters for angle in template matching)
	def twistCB(self,data):
		if data.angular.z is not 0:
			self.turning = time.time()
		self.signal.angular=data.angular
		self.signal.linear=data.linear




#Template matching callback
	def callback(self,data):

#	I used these to make sure we were only using new measurements, not needed anymore
#		if data.header.stamp.nsecs-self.lastMeasurement < 20000000:
#			return
#		self.lastMeasurement=data.header.stamp.nsecs




#magic
		self.ceilingFlag=True
		self.init=True
		self.start = time.time()
		if time.time()-self.turning > 2:
			self.angleRange=12
		else:
			self.angleRange=35



		template= self.bridge.imgmsg_to_cv2(data, "bgr8")
		(rows,cols) = template.shape[:2]

#		cv2.waitKey(1)
		template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		template = cv2.Canny(template, 150, 150)
		tempt=template

		found2 = None
#		postingImage = np.copy(image)
		resized=np.copy(imageCanny)

		if self.pose.position.x is not 0 and self.pose.position.y is not 0 and self.pose.position.x > mapSliceSize/2+1 and self.pose.position.y> mapSliceSize/2+1:
			resized=resized[(self.pose.position.y*2-mapSliceSize)/2:(self.pose.position.y*2-mapSliceSize)/2+mapSliceSize, (self.pose.position.x*2-mapSliceSize)/2:mapSliceSize+(self.pose.position.x*2-mapSliceSize)/2]

		edged=cv2.copyMakeBorder(resized,0,0,0,0,cv2.BORDER_REPLICATE)
#		print "Size: " + str(rows) + str(cols)		
		for scale in np.linspace(self.angle-self.angleRange, self.angle+self.angleRange,  10)[::-1]:
#			self.start = time.time()
			M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
			template = cv2.warpAffine(tempt,M,(cols,rows))
			template = template[(rows-sliceSize)/2:(rows-sliceSize)/2+sliceSize, (cols-sliceSize)/2:sliceSize+(cols-sliceSize)/2]
#                        self.start = time.time()
			result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
#                        del self.averageTime[0]
#                        self.averageTime.append(1/(time.time()-self.start))
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
			if found2 is None or maxVal > found2[0]:
				found2 = (maxVal, maxLoc, scale)
	                

#		print "Size: " + str(template.shape[:2])
#                self.pubImage.publish(self.bridge.cv2_to_imgmsg(template, "8UC1"))		
		(maxVal, maxLoc, maxScale) = found2
		(startX, startY) = (int(maxLoc[0]  +sliceSize/2), int(maxLoc[1]+sliceSize/2 ))
		(endX, endY) = (int((maxLoc[0] +sliceSize/2+40*math.sin(maxScale*0.0174533)) ), int((maxLoc[1] +sliceSize/2+40*math.cos(maxScale*0.0174533)) ))
		if self.pose.position.x is not startingX and self.pose.position.y is not startingY and self.pose.position.x > mapSliceSize/2+1 and self.pose.position.y> mapSliceSize/2+1:
			self.ceilingMeasurement.position.x=self.pose.position.x-mapSliceSize/2+startX
			self.ceilingMeasurement.position.y=self.pose.position.y-mapSliceSize/2+startY
#		else:
#			self.pose.position.x=startX
#			self.pose.position.y=startY
		self.ceilingAngle=maxScale

#		cv2.imshow("Image2", template)
#		cv2.imshow("Image3", edged)
#		cv2.waitKey(1)

#		self.t.header.stamp= rospy.Time.now()
#		self.t.header.frame_id = "map";
#		self.t.child_frame_id = "base_footprint";
#		self.t.transform.translation.x = self.pose.position.x/250.0;
#		self.t.transform.translation.y = self.pose.position.y/250.0;
#		self.q= tf.transformations.quaternion_from_euler(0, 0, (self.angle+90)*0.0174533)
#		self.t.transform.rotation.x = self.q[0]
#		self.t.transform.rotation.y = self.q[1]
#		self.t.transform.rotation.z = self.q[2]
#		self.t.transform.rotation.w = self.q[3]
#		self.br.sendTransform(self.t)
		self.ceilingConfidence=maxVal/60000000.

		if self.ceilingConfidence>1:
			self.ceilingConfidence=1.0

      	        del self.averageTime[0]
                self.averageTime.append(1/(time.time()-self.start))
#               if sum(self.averageTime) < 150:         
                print "Average Frequency: " + str(sum(self.averageTime)/5.)
                del self.averageConfidence[0]
                self.averageConfidence.append(self.ceilingConfidence)
                print "Average Confidence: " + str(sum(self.averageConfidence)/30.)

 #               print "Average Map (Edged): "+ str(np.average(edged))
#                print "Average template (Edged):"+str(np.average(template))


# fusion of measurements and output
	def kalman(self):
		while not rospy.is_shutdown():
#			self.pose.position.x+=math.sin(self.angle*0.0174533)*self.signal.linear.x*4.
#			self.pose.position.y+=math.cos(self.angle*0.0174533)*self.signal.linear.x*4.
#			self.angle+=self.signal.angular.z

			if self.ceilingFlag is True:
				if self.ceilingConfidence > thresholdConfidence:
					self.pose.position.x=self.pose.position.x+.05*self.ceilingConfidence*(-self.pose.position.x+self.ceilingMeasurement.position.x)
					self.pose.position.y=self.pose.position.y+.05*self.ceilingConfidence*(-self.pose.position.y+self.ceilingMeasurement.position.y)
					self.angle=self.angle+.4*(-self.angle+self.ceilingAngle)
#				print str(self.ceilingConfidence)
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

##UPDATE TRANSFORM MESSAGES
			self.t.header.stamp= rospy.Time.now()
			self.t.header.frame_id = "map";
			self.t.child_frame_id = "base_footprint";
			self.t.transform.translation.x = self.pose.position.x/450.*350./mapSliceSize;
			self.t.transform.translation.y = -self.pose.position.y/450.*350./mapSliceSize;
			self.q= tf.transformations.quaternion_from_euler(0, 0, -(self.angle-90)*0.0174533)
			self.t.transform.rotation.x = self.q[0]
			self.t.transform.rotation.y = self.q[1]
			self.t.transform.rotation.z = self.q[2]
			self.t.transform.rotation.w = self.q[3]
			self.br.sendTransform(self.t)
			self.counter=self.counter+1
#                        self.pubImage.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))
			if self.counter > 20:
				self.counter=0
                        	postingImage = np.copy(postingImage2)

                        	(startX, startY) = (int(self.pose.position.x), int(self.pose.position.y))
				(endX, endY) = (int((self.pose.position.x+mapSliceSize/350.*40*math.sin(self.angle*0.0174533)) ), int((self.pose.position.y+mapSliceSize/350.*40*math.cos(self.angle*0.0174533)) ))
                              	cv2.line(postingImage, ((startX-300)/2, (startY-300)/2), ((endX-300)/2, (endY-300)/2), (255, 255, 0), 2)
                        	cv2.circle(postingImage,((startX-300)/2,(startY-300)/2), int(mapSliceSize/350.*40), (255,255,0), int(mapSliceSize/350.*5))

                        	(startX, startY) = (int(self.odomMeasurement.position.x), int(self.odomMeasurement.position.y))
				(endX, endY) = (int((self.odomMeasurement.position.x+mapSliceSize/350.*40*math.sin(self.odomAngle*0.0174533)) ), int((self.odomMeasurement.position.y+mapSliceSize/350.*40*math.cos(self.odomAngle*0.0174533)) ))
                        	cv2.line(postingImage, ((startX-300)/2, (startY-300)/2),  ((endX-300)/2, (endY-300)/2), (0, 255, 0), 2)
                        	cv2.circle(postingImage,((startX-300)/2, (startY-300)/2), int(mapSliceSize/350.*40), (0,255,0), int(mapSliceSize/350.*5))

       	               	        if self.ceilingConfidence > thresholdConfidence:
					(startX, startY) = (int(self.ceilingMeasurement.position.x), int(self.ceilingMeasurement.position.y))
                	        	(endX, endY) = (int((self.ceilingMeasurement.position.x+mapSliceSize/350.*40*math.sin(self.ceilingAngle*0.0174533)) ), int((self.ceilingMeasurement.position.y+mapSliceSize/350.*40*math.cos(self.ceilingAngle*0.0174533)) ))
        	                	cv2.line(postingImage, ((startX-300)/2, (startY-300)/2),  ((endX-300)/2, (endY-300)/2), (0, 255, 255), 2)
 	                        	cv2.circle(postingImage,((startX-300)/2, (startY-300)/2), int(mapSliceSize/350.*40), (0,255,255), int(mapSliceSize/350.*5))

                	        (startX, startY) = (int(bucketX), int(bucketY))
                	        cv2.line(postingImage, ((startX-300)/2-10, (startY-300)/2), ((startX-300)/2+10, (startY-300)/2) , (200, 100, 200), 1)
        	                cv2.line(postingImage, ((startX-300)/2, (startY-300)/2-10),  ((startX-300)/2, (startY-300)/2+10), (200, 100, 200), 1)
	                        cv2.circle(postingImage,((startX-300)/2, (startY-300)/2), int(mapSliceSize/350.*30), (200,100,200), int(mapSliceSize/350.*5))

                                (startX, startY) = (int(waypointX*112.5), int(waypointY*112.5))
                                cv2.line(postingImage, ((startX-300)/2-10, (startY-300)/2), ((startX-300)/2+10, (startY-300)/2) , (200, 100, 200), 1)
                                cv2.line(postingImage, ((startX-300)/2, (startY-300)/2-10),  ((startX-300)/2, (startY-300)/2+10), (200, 100, 200), 1)
                                cv2.circle(postingImage,((startX-300)/2, (startY-300)/2), int(mapSliceSize/350.*30), (200,100,200), int(mapSliceSize/350.*5))

#                                postingImage=cv2.flip(imageCanny,1)

				postingImage=cv2.flip(postingImage,1)
				self.pubImage.publish(self.bridge.cv2_to_imgmsg(postingImage, "bgr8"))
#                                self.pubImage.publish(self.bridge.cv2_to_imgmsg(imageCanny, "8UC1"))

#			cv2.imshow("Image", postingImage)
			cv2.waitKey(1)
#			self.poseAndroid.header.stamp=10
#			self.poseAndroid.header.frame_id="leonardo"
#			self.poseAndroid.pose.position.x=self.t.transform.translation.x
#			self.poseAndroid.pose.position.y=self.t.transform.translation.y
#			self.poseAndroid.pose.orientation.x=self.t.transform.rotation.x
#			self.poseAndroid.pose.orientation.y=self.t.transform.rotation.y
#			self.poseAndroid.pose.orientation.z=self.t.transform.rotation.z
#			self.poseAndroid.pose.orientation.w=self.t.transform.rotation.w
#			self.pubPoseAndroid.publish(self.poseAndroid)



# main
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


