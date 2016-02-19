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

from cv2 import __version__
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
	print str(image.shape[0])
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	(rows,cols) = template.shape[:2]

class GL:


	def __init__(self):
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('chatter', String, queue_size=1)
		self.sub =rospy.Subscriber('/left_camera/image_raw', Image, self.callback)
		self.x=0
		self.y=0
		self.sx=0
		self.sy=0
		self.angle=0








	def callback(self,data):
		template= self.bridge.imgmsg_to_cv2(data, "bgr8")
		(rows,cols) = template.shape[:2]

		cv2.waitKey(1)
		template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		template = cv2.Canny(template, 175, 225)
		tempt=template

		found2 = None

		image = cv2.imread("/home/aaron/ceiling/t.png")
		resized=image;

		self.sx=self.x
		self.sy=self.y

		if self.x is not 0 and self.y is not 0 and self.x > 176 and self.y> 176:
			resized=resized[(self.y*2-350)/2:(self.y*2-350)/2+350, (self.x*2-350)/2:350+(self.x*2-350)/2]



		for scale in np.linspace(self.angle-45, self.angle+45,  5)[::-1]:
			M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
			template = cv2.warpAffine(tempt,M,(cols,rows))
			template = template[(rows-250)/2:(rows-250)/2+250, (cols-250)/2:250+(cols-250)/2]
			# if the resized image is smaller than the template, then break
			# from the loop

			# detect edges in the resized, grayscale image and apply template
			# matching to find the template in the image
			edged = cv2.Canny(resized, 25, 200)


			result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
			(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
			cv2.imshow("Image2", template)
			cv2.waitKey(1)
			# if we have found a new maximum correlation value, then ipdate
			# the bookkeeping variable
			if found2 is None or maxVal > found2[0]:
				found2 = (maxVal, maxLoc, scale)


		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(_, maxLoc, maxScale) = found2
		(startX, startY) = (int(maxLoc[0]  +125), int(maxLoc[1]+125 ))
		(endX, endY) = (int((maxLoc[0] +125+40*math.sin(maxScale*0.0174533)) ), int((maxLoc[1] +125+40*math.cos(maxScale*0.0174533)) ))

		# draw a bounding box around the detected result and display the image


		cv2.line(resized, (startX, startY), (endX, endY), (0, 0, 255), 2)
		cv2.circle(resized,(startX,startY), 40, (0,0,255), 5)




		if self.x is not 0 and self.y is not 0 and self.x > 176 and self.y> 176:
			#image=image[(self.y*2-350)/2:(self.y*2-350)/2+350, (self.x*2-350)/2:350+(self.x*2-350)/2]
			self.x+=-175+startX#-175+startX
			self.y+=-175+startY#175+startY
		else:
			self.x=startX#-175+startX
			self.y=startY#175+startY





		cv2.imshow("Image", image)
		cv2.imshow("Image2", template)
		cv2.imshow("Image3", resized)
		cv2.waitKey(1)






















def main(args):
	gl = GL()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)


