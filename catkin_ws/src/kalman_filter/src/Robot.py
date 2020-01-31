#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import inv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class Robot:
	def __init__(self):
		self.linear_velocity = [[1.0]]
		self.x_pred = float("inf")
		self.P_uncertainty = float("inf")

		self.x_initial = [[0]]
		self.p_initial = [[5000]]

		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		# self.pose = rospy.Subscriber('/pose', PoseStamped, self.getCoordinate)
		rospy.sleep(1)

	def scan_callback(self, data):
		front_scan = data.ranges[0]
		print(front_scan)
		"""front_scan_2 = data.ranges[num_measurements - 30:num_measurements]
		front_scan = front_scan_1 + front_scan_2

		for i in front_scan:
			if i != "inf":
				print(str(i) + "m")"""

	def getCoordinate(self, message):
		pass

	"""
	Kalman Filter Algorithm for State Estimation
	1. Predict: uses mathematical model to predict the current state of the robot
	2. Update given the prediction, updates the estimate and uncertainty
	"""
	# Prediction:
	def prediction(self, x_k_1, p_k_1, u_k, F, B, Q):
		x_priori = np.dot(F, x_k_1) + np.dot(B, u_k)
		prediction_priori = np.dot(np.dot(F, p_k_1), np.transpose(F)) + Q

		return (x_priori, prediction_priori)

	# Updating:
	def update(self, x_priori, prediction_priori, z_k, H, R):
		covariance = np.dot(np.dot(H, prediction_priori), np.transpose(H)) + R
		covariance_inverse = inv(covariance)

		kalman_gain = np.dot(np.dot(prediction_priori, np.transpose(H)), covariance_inverse)

		error = z_k - np.dot(H, x_priori)
		x_postori = x_priori + np.dot(kalman_gain, error)

		print("Error: " + str(error))

		J = np.eye(1) - np.dot(kalman_gain, H)

		prediction_postori = prediction_priori - np.dot(np.dot(np.dot(np.dot(prediction_priori, np.transpose(H)), covariance_inverse),H), prediction_priori)

		return (x_postori, prediction_postori)

	def start(self):
		print("Robot is starting up...")

		x_initial = 0
		p_initial = [[1000]]

		delta_t = 0.1 #1.0
		linear_velocity = 0.1
		F = [[1]]
		B = np.dot(delta_t, np.eye(1))
		Q = [[1]]

		z_k = 1.90  # observation
                H = [[1]]
                R = [[1]]

		x_priori, prediction_priori = self.prediction(x_initial, p_initial, linear_velocity, F, B, Q)
		x_postori, prediction_postori = self.update(x_priori, prediction_priori, z_k, H, R)

		print((x_priori, prediction_priori))
		print((x_postori, prediction_postori))

		while not rospy.is_shutdown():
			rospy.sleep(0.5)

		"""for i in range(0, 10):
		      z_k -= 0.10
  	              x_priori, prediction_priori = self.prediction(x_postori, prediction_postori, linear_velocity, F, B, Q)
	              x_postori, prediction_postori = self.update(x_priori, prediction_priori, z_k, H, R)
		      print("~~~ New Cycle: ~~~")
		      print("\n")
		      print("Observation: " + str(z_k) + "m")
		      print("Kalman Filter Output" + str(x_postori) + "m - with " + str(prediction_postori[0][0]) + " covariance")
		      print("\n")
		      # print((x_priori, prediction_priori))
                      # print((x_postori, prediction_postori))
		      time.sleep(2)"""

if __name__ == '__main__':
	rospy.init_node('kalman_filter')
	robot = Robot()
	robot.start()
