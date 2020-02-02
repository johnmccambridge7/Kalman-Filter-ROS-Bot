#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math

class Graph:
	def __init__(self):
		self.raw_file = open("calculated_coords.txt").read().split("\n")
		self.pose_file = open("kalman_with_pose_input_coords.txt").read().split("\n")
		self.cmd_vel_file = open("kalman_with_cmd_vel_input_coords.txt").read().split("\n")

		self.calc_coords = [] # raw coords from post topic
		self.pose_coords = [] # kalman filter with pose and scan input
		self.cv_coords = [] # kalman filter with cmd_vel and scan input
		# print(self.pose_file)

		for row in self.pose_file:
			if row != '':
				data = row.split("\t")
				if float(data[0]) > 0:
					self.pose_coords.append((data[0], data[1]))

		for row_2 in self.cmd_vel_file:
			if row_2 != '':
				data = row_2.split("\t")
				if float(data[0]) > 0:
					self.cv_coords.append((data[0], data[1]))


		for row_3 in self.raw_file:
			if row_3 != '':
				data = row_3.split("\t")
				if float(data[0]) > 0:
					self.calc_coords.append((data[0], data[1]))

	def draw(self):
		# draws the graph of the two kalman filters and the raw data
		print("Constructing output graph...")
		red = mpatches.Patch(color='red', label='KF with CMD VEL / LIDAR')
		blue = mpatches.Patch(color='blue', label='KF with Pose / LIDAR')
		green = mpatches.Patch(color='green', label='Raw coordinates from Pose')

		plt.plot(self.pose_coords, 'bo')
		plt.plot(self.calc_coords, 'go')
		plt.plot(self.cv_coords, 'ro')

		plt.legend(handles=[red, blue, green], loc='lower right')
		plt.show()
		pass

if __name__ == "__main__":
	g = Graph()
	g.draw()
