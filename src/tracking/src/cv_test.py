import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import rospy

def img_callback(image_head):
	bridge = CvBridge()

	try:
		cv_image = bridge.imgmsg_to_cv2(image_head, 'mono8')
	except CvBridgeError as e:
		print(e)
	
	# print(cv_image.shape)
	cv.imshow("Head Image window", cv_image)
	# gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
	cur_image_head = cv_image
	# cv.waitKey(3)

	########################### CIRCLES ########################
	# Example code values
	circles = get_circles(cur_image_head, 100, [1, 30], [100, 30])

	if circles is not None:
		print('found circles')
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
			# circle center
			cv.circle(cur_image_head, center, 1, (0, 100, 100), 3)
			# circle outline
			radius = i[2]
			cv.circle(cur_image_head, center, radius, (255, 0, 255), 3)

	cv.imshow("detected circles", cur_image_head)
	cv.waitKey(0)

"""Returns the x, y, and radius of the circles in the image given"""
"""Inputs:  img -- picture"""
"""			threshold -- int image threshold"""
"""			minmax -- tuple smallest and largest circles to detect"""
"""			minmax -- tuple thresholds of canny edge detector"""
def get_circles(img, threshold, minmax, cannythresh):

	# Turn image to grayscale
	# gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	gray = img
	# Blur the image
	# gray = cv.medianBlur(gray, 5)

	# Threshold the image
	# for i in range(len(gray)):
	# 	for j in range(len(gray[0])):
	# 		if gray[i][j] > threshold:
	# 			gray[i][j] = 255
	# 		else:
	# 			gray[i][j] = 0

	rows = gray.shape[0]
	# Dist between possible circle centers (want this to be big)
	mindist = rows

	circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, mindist,
                               param1=cannythresh[0], param2=cannythresh[1],
                               minRadius=minmax[0], maxRadius=minmax[1])

	return circles


def listener():
    rospy.Subscriber('/io/internal_camera/head_camera/image_raw', Twist, get_circles)
    rospy.spin()


if __name__ == "__main__":	
	# img_name = "target1.jpg"
	# src = cv.imread(img_name)

	rospy.init_node('image_converter', anonymous=True)
	image_sub_head = message_filters.Subscriber("/io/internal_camera/head_camera/image_raw", Image)
	ts = message_filters.TimeSynchronizer([image_sub_head], 1)
	ts.registerCallback(img_callback)
	
	rospy.spin() 
