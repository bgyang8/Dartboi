import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
import rospy

from tracking.transformation import get_transform

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
	
	traj_dist = 1  # [meters]
	head_camera_frame = 'head_camera'
	circle_frame = ... # TODO: find frame of circle center
	get_transform(head_camera_frame, circle_frame, traj_dist)
	
	cv.waitKey(0)

def get_circles(img, threshold, minmax, cannythresh):
	"""
	Returns the x, y, and radius of the circles in the image given
	Inputs: img -- picture
	 		threshold -- int image threshold
	 		minmax -- tuple smallest and largest circles to detect
	 		minmax -- tuple thresholds of canny edge detector
	"""
	gray = img
	rows = gray.shape[0]
	mindist = rows

	circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, mindist,
                               param1=cannythresh[0], param2=cannythresh[1],
                               minRadius=minmax[0], maxRadius=minmax[1])

	return circles

def listener():
    rospy.Subscriber('/io/internal_camera/head_camera/image_raw', Twist, get_circles)
    rospy.spin()


if __name__ == "__main__":	
	rospy.init_node('track_target', anonymous=True)
	image_sub_head = message_filters.Subscriber("/io/internal_camera/head_camera/image_raw", Image)
	ts = message_filters.TimeSynchronizer([image_sub_head], 1)
	ts.registerCallback(img_callback)
	
	rospy.spin() 
