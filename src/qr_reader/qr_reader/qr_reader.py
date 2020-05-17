#Node for reading QR-codes containing information about pose and orintation
import rclpy
from rclpy.node import Node

import tf2_msgs.msg
import geometry_msgs.msg
import time


class qr_pose_estimate(Node):
	def __init__(self):
		super().__init__('pose_estimate')
		self.msg = geometry_msgs.msg.TransformStamped()
		self.pub = self.create_publisher(geometry_msgs.msg.TransformStamped, "/tf", 1)
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.scan)
		self.time = Node.get_clock(self)
		self.time = self.get_clock().now().to_msg()
	
	def scan(self):

		self.msg.header.frame_id = "base_link"
		self.msg.child_frame_id = "body"
		self.msg.header.stamp = self.time
		self.msg.transform.translation.x = 2.0
		self.msg.transform.translation.y = 3.0
		self.msg.transform.translation.z = 4.0
		self.msg.transform.rotation.x = 2.0
		self.msg.transform.rotation.y = 3.0
		self.msg.transform.rotation.z = 4.0
		self.msg.transform.rotation.w = 1.0
		self.pub.publish(self.msg)
		
		self.get_logger().info("Pose x:")



def main(args=None):
	rclpy.init(args=args)
	pose_estimate = qr_pose_estimate()
	rclpy.spin(pose_estimate)

if __name__=="__main__":
	main()
