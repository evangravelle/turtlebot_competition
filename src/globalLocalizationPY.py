#!/usr/bin/env python
# USAGE
# python match.py --template cod_logo.png --images images

# import the necessary packages
import numpy as np
import rospy
from std_msgs.msg import String
import time
import sys
import glob
import cv2

from cv2 import __version__
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

print str(1)
# construct the argument parser and parse the arguments


# load the image image, convert it to grayscale, and detect edges
#template = cv2.imread(args["template"])
template = cv2.imread("/home/aaron/ceiling/test.png")
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
template = cv2.Canny(template, 50, 200)
(tH, tW) = template.shape[:2]
#cv2.imshow("Template", template)

# loop over the images to find the template in
for imagePath in np.linspace(0, 1,  1)[::-1]:


	# load the image, convert it to grayscale, and initialize the
	# bookkeeping variable to keep track of the matched region
	#image = cv2.imread(imagePath)
	image = cv2.imread("/home/aaron/ceiling/t.png")
	no=image
	print str(image.shape[0])
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	(rows,cols) = template.shape[:2]
	found = None
	start = time.clock()


	# loop over the scales of the image
	for scale in np.linspace(0, 1,  1)[::-1]:
		
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
#		resized = imutils.resize(gray, width = int(gray.shape[1] * scale))

		resized=gray;

		M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
		templateTemp = cv2.warpAffine(template,M,(cols,rows))
		templateTemp = templateTemp[(cols-.707*cols)/2:(cols-.707*cols)/2+.707*cols, (rows-.707*rows)/2:.707*rows+(rows-.707*rows)/2]
		r = gray.shape[1] / float(resized.shape[1])

		# if the resized image is smaller than the template, then break
		# from the loop
		if resized.shape[0] < tH or resized.shape[1] < tW:
			break

		# detect edges in the resized, grayscale image and apply template
		# matching to find the template in the image
		edged = cv2.Canny(resized, 50, 200)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

		# if we have found a new maximum correlation value, then ipdate
		# the bookkeeping variable
		if found is None or maxVal > found[0]:
			found = (maxVal, maxLoc, r, scale)
		print str(scale) +" : "+ str(maxVal)

	# unpack the bookkeeping varaible and compute the (x, y) coordinates
	# of the bounding box based on the resized ratio
	(_, maxLoc, r, maxScale) = found
	(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
	(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))

	# draw a bounding box around the detected result and display the image

	elapsed = time.clock() - start	



	print str(edged.shape[0])

class GL:


	def __init__(self):
		self.bridge = CvBridge()
		self.pub = rospy.Publisher('chatter', String, queue_size=1)
		self.sub =rospy.Subscriber('/left_camera/image_raw', Image, self.callback)

	def callback(self,data):
		template= self.bridge.imgmsg_to_cv2(data, "bgr8")
		template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
		template = cv2.Canny(template, 50, 200)
		found2 = None

		image = cv2.imread("/home/aaron/ceiling/t.png")
		resized=gray;

		M = cv2.getRotationMatrix2D((cols/2,rows/2),scale,1)
		templateTemp = cv2.warpAffine(template,M,(cols,rows))
		templateTemp = templateTemp[(cols-.707*cols)/2:(cols-.707*cols)/2+.707*cols, (rows-.707*rows)/2:.707*rows+(rows-.707*rows)/2]
		r = gray.shape[1] / float(resized.shape[1])

		# if the resized image is smaller than the template, then break
		# from the loop

		# detect edges in the resized, grayscale image and apply template
		# matching to find the template in the image
		edged = cv2.Canny(resized, 50, 200)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)

		# if we have found a new maximum correlation value, then ipdate
		# the bookkeeping variable
		if found2 is None or maxVal > found[0]:
			found2 = (maxVal, maxLoc, r, scale)
		print str(scale) +" : "+ str(maxVal)

		# unpack the bookkeeping varaible and compute the (x, y) coordinates
		# of the bounding box based on the resized ratio
		(_, maxLoc, r, maxScale) = found2
		(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
		(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))

		# draw a bounding box around the detected result and display the image
		cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
		elapsed = time.clock() - start
		cv2.putText(image,str(maxScale),(10,500), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2)
		cv2.imshow("Image", image)
		cv2.waitKey(10)

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


