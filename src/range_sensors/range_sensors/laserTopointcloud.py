#!/usr/bin/env python
#Convert laserscan message from range sensors to point cloud

import rclpy
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from geometry_msgs.msg import Point

import serial
import math
import time
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry

class laserToPointcloud(Node):
	def __init__(self):
		super().__init__('laser_to_pointcloud')
		self.subscription = self.create_subscription(LaserScan, '/agv/sonar_to_laser1', self.scan, 10)
		self.subscription
		lp = lg.LaserProjection()
		self.msg = lp.projectlaser()

	def scan(self, msg):
		self.get_logger().info('I heard: "%s"' % msg.ranges)

def main(args=None):
	rclpy.init(args=args)
	laser_to_pointcloud = laserToPointcloud()
	rclpy.spin(laser_to_pointcloud)
	laser_to_pointcloud.destroy_node()
	rclpy.shutdown()
	
