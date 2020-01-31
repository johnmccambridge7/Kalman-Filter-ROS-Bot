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
		self.linear_velocity = 1.0
		self.x_pred = float("inf")
		self.P_uncertainty = float("inf")

		self.x_initial = 0
		self.p_initial = [[1000]]
		self.distance_to_wall = 2.0
		self.initial_z = 0.0

		delta_t = 0.1

                self.F = [[1]]
                self.B = np.dot(delta_t, np.eye(1))
                self.Q = [[1]]

                # z_k = 2.0 - self.distance_to_wall  # observation
                self.H = [[1]]
                self.R = [[1]]

		# get the initial estimates of the kalman filter setup
		self.x_priori, self.prediction_priori = self.prediction(self.x_initial, self.p_initial, self.linear_velocity, self.F, self.B, self.Q)
                self.x_postori, self.prediction_postori = self.update(self.x_priori, self.prediction_priori, self.initial_z, self.H, self.R)

		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		self.pose = rospy.Subscriber('/pose', PoseStamped, self.getCoordinate)

		rospy.sleep(1)

	def scan_callback(self, data):
		front_scan = data.ranges[0]
		if front_scan < 2.0:
			self.distance_to_wall = front_scan
			# start kalman filter prediction
			print(self.distance_to_wall)

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

		# print("Error: " + str(error))

		J = np.eye(1) - np.dot(kalman_gain, H)

		prediction_postori = prediction_priori - np.dot(np.dot(np.dot(np.dot(prediction_priori, np.transpose(H)), covariance_inverse),H), prediction_priori)

		return (x_postori, prediction_postori)

	def start(self):
		print("Robot is starting up...")

		z_k = 2.0 - self.distance_to_wall  # observation

		# print("~ Initial Cycle ~")
                # print((self.x_priori, self.prediction_priori))
                # print((self.x_postori, self.prediction_postori))

		# self.x_priori, self.prediction_priori = self.prediction(self.x_postori, self.prediction_postori, self.linear_velocity, self.F, self.B, self.Q)
		# self.x_postori, self.prediction_postori = self.update(self.x_priori, self.prediction_priori, z_k, self.H, self.R)

		cycle = 1

		while not rospy.is_shutdown():
	                z_k = 2.0 - self.distance_to_wall  # observation
			print("")
			print("~ Cycle: "+ str(cycle) +"  ~ " + str(z_k))
			self.x_priori, self.prediction_priori = self.prediction(self.x_postori, self.prediction_postori, self.linear_velocity, self.F, self.B, self.Q)
                	self.x_postori, self.prediction_postori = self.update(self.x_priori, self.prediction_priori, z_k, self.H, self.R)

			print("X Calc: " + str(z_k) + "m")
			print("X Estimation: " + str(self.x_postori[0][0]) + "m - covariance: " + str(self.prediction_postori[0][0]))

			cycle += 1
			# print((self.x_priori, self.prediction_priori))
                	# print((self.x_postori, self.prediction_postori))
			# print("")
			# cycle += 1
			rospy.sleep(0.5)

if __name__ == '__main__':
	rospy.init_node('kalman_filter')
	robot = Robot()
	robot.start()
