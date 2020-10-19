#! /usr/bin/env python
# -*- coding:utf-8 -*-




import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cormodule

import cv2.aruco as aruco
import sys

#--- Define Tag de teste
id_to_find  = 13
marker_size  = 10 #- [cm]

#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


#--- Get the camera calibration path
calib_path  = "/home/borg/catkin_ws/src/robot202/ros/exemplos202/scripts/"
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_raspi.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_raspi.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
parameters  = aruco.DetectorParameters_create()

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

bridge = CvBridge()
cv_image = None



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")

	try:
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		
		if ids is not None and ids[0] == id_to_find:
			#-- ret = [rvec, tvec, ?]
			#-- array of rotation and position of each marker in camera frame
			#-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
			#-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
			ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

			#-- Unpack the output, get only the first
			rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

			#-- Draw the detected marker and put a reference frame over it
			aruco.drawDetectedMarkers(cv_image, corners)
			aruco.drawAxis(cv_image, camera_matrix, camera_distortion, rvec, tvec, 10)

			#-- Print the tag position in camera frame
			str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
			cv2.putText(cv_image, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#-- Obtain the rotation matrix tag->camera
			R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
			R_tc    = R_ct.T

			#-- Get the attitude in terms of euler 321 (Needs to be flipped first)
			roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

			#-- Print the marker's attitude respect to camera frame
			str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
								math.degrees(yaw_marker))
			cv2.putText(cv_image, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


			#-- Now get Position and attitude f the camera respect to the marker
			pos_camera = -R_tc*np.matrix(tvec).T

			str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
			cv2.putText(cv_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

			#-- Get the attitude of the camera respect to the frame
			roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
			str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
								math.degrees(yaw_camera))
			cv2.putText(cv_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


		cv2.imshow("Camera", cv_image)
		cv2.waitKey(1)
	except CvBridgeError as e:
		print('ex', e)
	
if __name__=="__main__":
	rospy.init_node("aruco")

	topico_imagem = "/camera/image/compressed"


	recebe_imagem = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

	try:

		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			velocidade_saida.publish(vel)
			rospy.sleep(0.1)

	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
