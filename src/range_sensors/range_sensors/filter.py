#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
import time

#Single-state Kalman-filter
class KalmanFilter(object):
	def __init__(self, Q = 1e-3, R = 1, H = 1, A = 1, x = 0.0, P = 1.0, B = None):
		self.A = A
		self.B = B
		self.H = H
		self.current_state_estimate = x
		self.current_prob_estimate = P
		self.Q = Q
		self.R = R
	
	def step(self, measurement, control = 0):
		priori_state_estimate = self.A * self.current_state_estimate
		priori_prob_estimate = (self.A * self.current_prob_estimate) * self.A + self.Q

		kalman_gain = priori_prob_estimate / (priori_prob_estimate + self.R)

		self.current_state_estimate = priori_state_estimate + kalman_gain * (measurement - (self.H * priori_state_estimate))

		self.current_prob_estimate = (1 - (kalman_gain * self.H)) * priori_state_estimate

	def current_state(self):
		return self.current_state_estimate

class subscriber(Node):

	def __init__(self):
		super().__init__('sensor_subsciber')
		self.subscription = self.create_subscription(Range, 
			'/agv/sonar_1',
			self.listener_callback,
			10)
		self.subscription
		
	def filtered_values(self, message):
		kalman = KalmanFilter()
		kalman.step(message.range)
		return kalman.current_state()

	def listener_callback(self, message):
		msg = self.get_logger().info('%s' %message.range)
		msg = float(msg)
		kalman = KalmanFilter()
		val = kalman.step(msg)
		#val = self.filtered_values(msg)
		#self.get_logger().info('I heard: %s %s' val)
		print('raw: ', msg)

def main(args=None):
	rclpy.init(args=args)
	sensor_subscriber = subscriber()
	rclpy.spin(sensor_subscriber)

if __name__ == '__main__':
	main()










