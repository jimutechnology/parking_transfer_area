#!/usr/bin/env python
# This node is to check whether the ground is clear

import threading 
import time
import numpy 
import rospy
import copy

from area_common import AreaParam
from area_common import ServiceNode, RunService
from sensor_msgs.msg import LaserScan
from transfer_area.srv import getGroundStatus, getGroundStatusResponse

class GroundNode(ServiceNode):
	def __init__(self):
		ServiceNode.__init__(self)
		param = AreaParam()
		self.setRate(0.1)
		# thread locks
		self.lock_get_ground = threading.Lock()
		self.lock_scan_data = threading.Lock()
		
		self.Subscriber("scan", LaserScan, self.scan_rx, queue_size=1)
		s = rospy.Service('get_ground_status', getGroundStatus, self.check_ground)
		self.to_scan_ground = False
		self.b_lidar1_scanned = False
		self.b_lidar2_scanned = False

		# parameters
		self.frame_id_1 = rospy.get_param("lidar_1/frame_id")
		self.frame_id_2 = rospy.get_param("lidar_2/frame_id")
		self.lidar_pose_1 = rospy.get_param("lidar_1/pose")
		self.lidar_pose_2 = rospy.get_param("lidar_2/pose")
		self.max_ground_x = param.max_ground_x
		self.min_ground_x = param.min_ground_x
		self.max_ground_y = param.max_ground_y
		self.min_ground_y = param.min_ground_y

		# TODO
		# self.scan_1 = 
		# self.scan_2 = 

	def scan_rx(self, data):
		with self.lock_scan_data:
			if self.to_scan_ground:
				# if lidar1 hasn't scanned the wheel, get the data if it's from lidar1
				if not self.b_lidar1_scanned:
					if data.header.frame_id == self.frame_id_1:
						self.scan_1 = copy.deepcopy(data)
						self.b_lidar1_scanned = True
				# if lidar2 hasn't scanned the wheel, get the data if it's from lidar2
				if not self.b_lidar2_scanned:
					if data.header.frame_id == self.frame_id_2:
						self.scan_2 = copy.deepcopy(data)
						self.b_lidar2_scanned = True


	def check_ground(self, req):
		# req is empty
		with self.lock_get_ground:
			with self.lock_scan_data:
				self.to_scan_ground = True
				self.b_lidar1_scanned = False
				self.b_lidar2_scanned = False

			# waiting for data
			while not (self.b_lidar1_scanned and self.b_lidar2_scanned):
				time.sleep(0.01)

			# now we have scans from both lidars
			with self.lock_scan_data:
				b_ground_clear = self.process_scans()

			return getGroundStatusResponse(b_ground_clear)

	# @ret: whether ground is clear
	def process_scans(self):
		# Lidar 1, upside-down
		phi = self.scan_1.angle_min
		phi_step = self.scan_1.angle_increment
		l_x = self.lidar_pose_1[0]
		l_y = self.lidar_pose_1[1]
		l_yaw = self.lidar_pose_1[2]
		for r in self.scan_1.ranges:
			if numpy.isinf(r):
				phi = phi + phi_step
				continue

			# 1. Transform the point from lidar frame to inertial frame
			x_l = numpy.cos(-phi) * r
			y_l = numpy.sin(-phi) * r
			x = x_l * numpy.cos(l_yaw) - y_l * numpy.sin(l_yaw) + l_x
			y = x_l * numpy.sin(l_yaw) + y_l * numpy.cos(l_yaw) + l_y
			phi = phi + phi_step
			# 2. Check if this point is in the ground area
			if x < self.max_ground_x and x > self.min_ground_x and y < self.max_ground_y and y > self.min_ground_y:
				print x, self.max_ground_x, self.min_ground_x, y, self.max_ground_y, self.min_ground_y
				print x<self.max_ground_x, x>self.min_ground_x, y<self.max_ground_y, y>self.min_ground_y
				print "1- ", x_l, y_l, "; ", x, y, "; ", phi, r
				print l_x, l_y, l_yaw
				return False

		# Lidar 2, upside-down
		phi = self.scan_2.angle_min
		phi_step = self.scan_2.angle_increment
		l_x = self.lidar_pose_2[0]
		l_y = self.lidar_pose_2[1]
		l_yaw = self.lidar_pose_2[2]
		for r in self.scan_2.ranges:
			if numpy.isinf(r):
				phi = phi + phi_step
				continue

			# 1. Transform the point from lidar frame to intertial frame
			x_l = numpy.cos(-phi) * r
			y_l = numpy.sin(-phi) * r
			x = x_l * numpy.cos(l_yaw) - y_l * numpy.sin(l_yaw) + l_x
			y = x_l * numpy.sin(l_yaw) + y_l * numpy.cos(l_yaw) + l_y
			phi = phi + phi_step
			# 2. Check if this point is in the ground area
			if x < self.max_ground_x and x > self.min_ground_x and y < self.max_ground_y and y > self.min_ground_y:
				print x, self.max_ground_x, self.min_ground_x, y, self.max_ground_y, self.min_ground_y
				print x<self.max_ground_x, x>self.min_ground_x, y<self.max_ground_y, y>self.min_ground_y
				print "2- ", x_l, y_l, "; ", x, y, "; ", phi, r
				print l_x, l_y, l_yaw
				return False

		# If no objects are detected, return True
		return True

if __name__ == '__main__':
	RunService(GroundNode)
