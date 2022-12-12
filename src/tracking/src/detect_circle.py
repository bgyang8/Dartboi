import os
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import rospy

from transformation import get_transform

DIST_RADIUS_SLOPE = -0.0469
DIST_RADIUS_OFFSET = 4.14

X_SCALE_SLOPE = 0.347
X_SCALE_OFFSET = 0.944

Y_SCALE_SLOPE = -1.9
Y_SCALE_OFFSET = 3.6


def img_callback(image_head, camera_info):
	# print(type(camera_info))

	bridge = CvBridge()

	K_matrix = np.reshape(camera_info.K, (3, 3))

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

	N = 10
	averaged_radius = 0
	averaged_x = 0
	averaged_y = 0

	max_radii = []


	for i in range(N):
		
		circles = get_circles(cur_image_head, 70, [10, 80], [100, 30])

		if circles is not None:
			# print('found circles')
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])
				# circle center
				cv.circle(cur_image_head, center, 1, (0, 100, 100), 3)
				# circle outline
				radius = i[2]
				cv.circle(cur_image_head, center, radius, (255, 0, 255), 3)

			circle = circles[0][0]

			averaged_x += circle[0]/N
			averaged_y += circle[1]/N
			averaged_radius += circle[2]/N
			max_radii.append(circle[2])

	max_radius = max(max_radii)
	print('radius')
	print(averaged_radius)
	print(max_radius)
	print(get_coords_from_circle(averaged_x, averaged_y, max_radius, K_matrix))
		# print(get_coords_from_circle(circle[0], circle[1], circle[2], K_matrix))

	cv.imshow("detected circles", cur_image_head)
	


	# traj_dist = 1  # [meters]
	# head_camera_frame = 'head_camera'
	# circle_frame = ... # TODO: find frame of circle center
	# get_transform(head_camera_frame, circle_frame, traj_dist)
	
	cv.waitKey(0)


def get_coords_from_circle(x, y, radius, K):

	lam = DIST_RADIUS_SLOPE * radius + DIST_RADIUS_OFFSET

	# print('dist')
	# print(lam)
	uv1 = np.array([x, y, 1]).reshape((1,3))

	coords = (lam*uv1)@np.linalg.inv(K)
	# print(coords)

	coords = coords[0]
	coords[0] = X_SCALE_SLOPE * coords[0] + X_SCALE_OFFSET
	coords[1] = Y_SCALE_SLOPE * coords[1] + Y_SCALE_OFFSET
	coords[2] = lam

	return coords

def get_circles(img, threshold, minmax, cannythresh):
	"""
	Returns the x, y, and radius of the circles in the image given
	Inputs: img -- picture
	 		threshold -- int image threshold
	 		minmax -- tuple smallest and largest circles to detect
	 		minmax -- tuple thresholds of canny edge detector
	"""

	gray = img

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

	return circles

if __name__ == "__main__":	
	rospy.init_node('track_target', anonymous=True)
	image_sub_head = message_filters.Subscriber("/io/internal_camera/head_camera/image_raw", Image)
	camera_info_head = message_filters.Subscriber("/io/internal_camera/head_camera/camera_info", CameraInfo)

	ts = message_filters.TimeSynchronizer([image_sub_head, camera_info_head], 10)
	ts.registerCallback(img_callback)
	print("finished setup")
	
	rospy.spin() 
