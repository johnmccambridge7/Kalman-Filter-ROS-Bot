#!/usr/bin/env python
import rospy
import numpy as np
import maplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped

class Robot:
	def __init__(self):
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
		self.pose = rospy.Subscriber('/pose', PoseStamped, self.getCoordinate)

	def getCoordinate(self, message):
		pass

	def start(self):
		print("Robot is starting up...")
		while not rospy.is_shutdown():
			print("Robot is running...")
			rospy.sleep(0.1)
