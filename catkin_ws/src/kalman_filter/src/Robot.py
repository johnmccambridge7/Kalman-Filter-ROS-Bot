#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import inv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

class Robot:
	def __init__(self):
		self.linear_velocity = 0.2
		self.x_pred = float("inf")
		self.P_uncertainty = float("inf")

		self.x_initial = 0
		self.p_initial = [[1000]]
		self.distance_to_wall = 2.0
		self.initial_z = 0.0

		self.delta_t = 0.1
		self.current_time = rospy.get_time()

                self.F = [[1]]
                self.B = np.dot(self.delta_t, np.eye(1))
                self.Q = [[1]]

                self.H = [[1]]
                self.R = [[1]]

		# get the initial estimates of the kalman filter setup
		self.x_priori, self.prediction_priori = self.prediction(self.x_initial, self.p_initial, self.linear_velocity, self.F, self.B, self.Q)
                self.x_postori, self.prediction_postori = self.update(self.x_priori, self.prediction_priori, self.initial_z, self.H, self.R)

		self.cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
		self.scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		self.pose = rospy.Subscriber('/pose', PoseStamped, self.getCoordinate)

		self.estimated_pose = rospy.Publisher('/estimated_pose', PoseWithCovarianceStamped, queue_size=0)

		rospy.sleep(1)

	def velocity_callback(self, data):
		self.linear_velocity = data.linear.x

	def scan_callback(self, data):
		front_scan = data.ranges[0]
		if front_scan < 2.0:
			self.distance_to_wall = front_scan
			# print(self.distance_to_wall)

	def getCoordinate(self, message):
		self.delta_t = (rospy.get_time() - self.current_time)
		# print("delta t: " + str(self.delta_t))
		self.current_time = rospy.get_time()

	def publish_estimate_message(self, information):
		ps = PoseWithCovarianceStamped()
		ps.header.frame_id = 'map'
		ps.header.stamp = rospy.Time.now()

		position = information['pose']
		covariance = information['covariance']

		ps.pose.pose.position.x = position['x']
		ps.pose.pose.position.y = position['y']
		ps.pose.pose.position.z = position['z']

		ps.pose.covariance = [covariance,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # covariance

		self.estimated_pose.publish(ps)

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

		cycle = 1

		while not rospy.is_shutdown():
	                z_k = 2.0 - self.distance_to_wall  # observation
			print("")
			print("~ Cycle: "+ str(cycle) +"  ~ " + str(z_k))
			self.x_priori, self.prediction_priori = self.prediction(self.x_postori, self.prediction_postori, self.linear_velocity, self.F, self.B, self.Q)
                	self.x_postori, self.prediction_postori = self.update(self.x_priori, self.prediction_priori, z_k, self.H, self.R)

			print("X Calc: " + str(z_k) + "m")
			print("X Estimation: " + str(self.x_postori[0][0]) + "m - covariance: " + str(self.prediction_postori[0][0]))

			node_information = {
				"pose": {
					"x": self.x_postori[0][0],
					"y": 0,
					"z": 0
				},
				"covariance": [self.prediction_postori[0][0]]
			}

			self.publish_estimate_message(node_information)

			cycle += 1

			# print((self.x_priori, self.prediction_priori))
                	# print((self.x_postori, self.prediction_postori))
			# print("")
			rospy.sleep(0.5)

if __name__ == '__main__':
	rospy.init_node('kalman_filter')
	robot = Robot()
	robot.start()
