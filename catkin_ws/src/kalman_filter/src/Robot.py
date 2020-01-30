#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class Robot:
	def __init__(self):
		self.linear_velocity = [[1.0]]
		self.x_pred = float("inf")
		self.P_uncertainty = float("inf")

		self.x_initial = [[0]]
		self.p_initial = [[1000]]

		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.pose = rospy.Subscriber('/pose', PoseStamped, self.getCoordinate)

	def getCoordinate(self, message):
		pass

	"""
	Kalman Filter Algorithm for State Estimation
	1. Predict: uses mathematical model to predict the current state of the robot
	2. Update given the prediction, updates the estimate and uncertainty
	"""
	# Prediction:
	def prediction(self, x_k_1, p_k_1, u_k, A, B, Q):
		x_priori = np.dot(A, x_k_1) + np.dot(B, u_k)
		prediction_priori = np.dot(np.dot(A, p_k_1), np.transpose(A)) + Q

		return (x_priori, prediction_priori)

	# Updating:
	def update(self, x_priori, prediction_priori, z_k, C, R):
		kalman_gain = np.dot(prediction_priori, np.transpose(C)) + 1/(np.dot(np.dot(C, prediction_priori), np.transpose(C)) + R)
		x_postori = x_priori + np.dot(kalman_gain, (z_k - np.dot(C, x_priori)))
		prediction_postori = prediction_priori + np.dot(np.dot(kalman_gain, C), prediction_priori)

		return (x_postori, prediction_postori)

	def start(self):
		print("Robot is starting up...")

		x_initial = 0
		p_initial = [[1000]]

		delta_t = 1.0
		linear_velocity = 1.0
		A = [[1]]
		B = np.dot(delta_t, np.eye(1))
		Q = [[1]]

		x_priori, prediction_priori = self.prediction(x_initial, p_initial, linear_velocity, A, B, Q)

		z_k = 0.5
		C = [[1]]
		R = [[1]]

		x_postori, prediction_postori = self.update(x_priori, prediction_priori, C, R)
		# print(self.prediction([[0]], [[1000]], [[1.0]], [[1]], [[0]], [[0.5]]))
		"""while not rospy.is_shutdown():
			print("Robot is running...")
			rospy.sleep(0.1)"""

if __name__ == '__main__':
	rospy.init_node('kalman_filter')
	robot = Robot()
	robot.start()
