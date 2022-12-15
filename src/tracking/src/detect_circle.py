import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import rospy
import time
from geometry_msgs.msg import Twist, Vector3

from transformation import get_transform

DIST_RADIUS_SLOPE = -0.0268#-0.0469
DIST_RADIUS_OFFSET = 2.876 #  4.14

X_SCALE_SLOPE = 1
X_SCALE_OFFSET = 0

Y_SCALE_SLOPE = -1.9
Y_SCALE_OFFSET = -0.74


def img_callback(image_head, camera_info):

	pub = rospy.Publisher('circle_transform', Twist, queue_size=10)


	# print(type(camera_info))

	bridge = CvBridge()

	K_matrix = np.reshape(camera_info.K, (3, 3))
	# K_matrix = np.eye(3)


	try:
		cv_image = bridge.imgmsg_to_cv2(image_head, 'mono8')
		# cv_image = bridge.imgmsg_to_cv2(image_head, 'bgr8')

		# cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

	except CvBridgeError as e:
		print(e)
	
	# cv_image[:,:,0] = 0
	# cv_image[:,:,1] = 0
	# red_img = cv_image[:,:,2]
	# a,red_img = cv.threshold(cv_image[:,:,2], 90,255,cv.THRESH_TOZERO)
	# cv_image[:,:,2] = red_img

	_,thresh = cv.threshold(cv_image,40,255,cv.THRESH_TOZERO)
	# _,thresh = cv.threshold(cv_image,0,200,cv.THRESH_TOZERO_INV)
	thresh[thresh>180] = 255;

	# print(cv_image[100,100,:])
	# # print(cv_image.shape)
	cv.imshow("Head Image window" ,thresh)
	# cv.imshow("Head Image window 2", cv_image[:,:,1])
	# cv.imshow("Head Image window 3", cv_image[:,:,2])
	# cv.waitKey(50)

	# time.sleep(30)
	# gray_image = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
	cur_image_head = thresh
	# print("yhere")


	########################### CIRCLES ########################
	# Example code values

	N = 10
	averaged_x = 0
	averaged_y = 0

	all_radii = []

	

	for i in range(N):
		
		circles = get_circles(cur_image_head, 70, [20, 70], [60,40])

		if circles is not None:
			# print('found circles')
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])
				# circle center
				cv.circle(cur_image_head, center, 1, (0, 100, 100), 3)
				# circle outline
				radius = i[2]
				cv.circle(cur_image_head, center, radius, (255, 0, 255), 1)

			circle = circles[0][0]

			averaged_x += circle[0]/N
			averaged_y += circle[1]/N
			all_radii.append(circle[2])

	if all_radii:
		max_radius = max(all_radii)
		min_radius = min(all_radii)
		avg_radius = np.mean(all_radii)
		median_radius = np.median(all_radii)

		cv.circle(cur_image_head, (int(averaged_x), int(averaged_y)), 0, (255, 0, 0), 3)


		# print('radius')
		print(min_radius, max_radius,avg_radius,median_radius)
		# print(averaged_radius)
		# print(max_radius)
		coords = get_coords_from_circle(averaged_x, averaged_y, avg_radius, K_matrix)

		print(coords)
			# print(get_coords_from_circle(circle[0], circle[1], circle[2], K_matrix))

		cv.imshow("detected circles", cur_image_head)	


		#####PUBLISH CODE:
		circle_twist = Twist()
		circle_twist.linear.x = coords[0]
		circle_twist.linear.y = coords[1]
		circle_twist.linear.z = coords[2]
		circle_twist.angular.x = 0
		circle_twist.angular.y = 0
		circle_twist.angular.z = 0
		pub.publish(circle_twist)


	# traj_dist = 1  # [meters]
	# head_camera_frame = 'head_camera'
	# circle_frame = ... # TODO: find frame of circle center
	# get_transform(head_camera_frame, circle_frame, traj_dist)
	print('ready')
	cv.waitKey(0)


def get_coords_from_circle(x, y, radius, K):
	# print(np.linalg.inv(K))
	lam = DIST_RADIUS_SLOPE * radius + DIST_RADIUS_OFFSET

	# print('dist')
	# print(lam)
	uv1 = np.array([x, y, 1]).reshape((3,1))

	coords = np.linalg.inv(K)@(lam*uv1)
	# print(coords)

	# coords = coords[0]
	coords[0] = X_SCALE_SLOPE * coords[0] + X_SCALE_OFFSET
	coords[1] = Y_SCALE_SLOPE * coords[1] + Y_SCALE_OFFSET
	# coords[2] = coords[2]

	return coords

def get_circles(img, threshold, minmax, cannythresh):
	"""
	Returns the x, y, and radius of the circles in the image given
	Inputs: img -- picture
	 		threshold -- int image threshold
	 		minmax -- tuple smallest and largest circles to detect
	 		minmax -- tuple thresholds of canny edge detector
	"""
	if len(img.shape) == 3:
		img = img[:, :, 2]
	gray = img
	# print(gray.shape)
	# cv.imshow("hm",gray)
	# cv.waitKey(30)

	# for i in range(len(img)):
	# 	for j in range(len(img[0])):
	# 		if img[i][j] > threshold:
	# 			img[i][j] = 255
	# 		else:
	# 			img[i][j] = 0

	# gray = cv.blur(gray, (5, 5))



	rows = gray.shape[0]
	mindist = rows

	circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, mindist,
                               param1=cannythresh[0], param2=cannythresh[1],
                               minRadius=minmax[0], maxRadius=minmax[1])

	# print(len(circles))
	return circles

if __name__ == "__main__":	
	rospy.init_node('track_target', anonymous=True)
	image_sub_head = message_filters.Subscriber("/io/internal_camera/head_camera/image_raw", Image)
	camera_info_head = message_filters.Subscriber("/io/internal_camera/head_camera/camera_info", CameraInfo)

	ts = message_filters.TimeSynchronizer([image_sub_head, camera_info_head], 10)
	# ts = message_filters.TimeSynchronizer([image_sub_head], 10)
	ts.registerCallback(img_callback)
	print("finished setup")
	
	rospy.spin() 
