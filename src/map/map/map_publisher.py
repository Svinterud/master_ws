#Node for publishing pre-loaded occupancy grid maps

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np


class Map(Node):
	def __init__(self,origion_x=0.0,origion_y=0.0,resolution=0.1, width=50, height=50):
		super().__init__('map_pub')
		self.msg = OccupancyGrid()
		#self.pub = self.create_publisher(OccupancyGrid)
		#timer_period = 0.05
		#self.timer = self.create_timer(timer_period, self.scan)
		self.time = Node.get_clock(self)
		self.time = self.get_clock().now().to_msg()
		self.orogion_x = origion_x
		self.origion_y = origion_y
		self.resolution = resolution
		self.width = width
		self.height = height
		self.grid = np.zeros((height, width))

		self.time = Node.get_clock(self)
		self.time = self.get_clock().now().to_msg()

	def to_message(self):
		#Header
		self.msg.header.stamp = self.time
		self.msg.header.frame_id = "map"
		#MapMetaData
		self.msg.info.resolution = self.resolution
		self.msg.info.width = self.width
		self.msg.info.height = self.height
		
		flat_grid = self.grid.reshape((self.grid.size))*100
		self.msg.data = list(np.round(flat_grid))
		return self.msg

	def set_cell(self, x, y, val):
		pass

class Mapper(Node):

    
	def __init__(self):
		#super().__init__('mapper')

		self._map = Map()
		self._map_pub = self.create_publisher('map', OccupancyGrid,1)
		self._map_data_pub = self.create_publisher('map_metadata', MapMetaData)


	def scan_callback(self, scan):

        
		# Fill some cells in the map just so we can see that something is 
		# being published. 
		self._map.grid[0, 0] = 1.0
		self._map.grid[0, 1] = .9
		self._map.grid[0, 2] = .7
		self._map.grid[1, 0] = .5
		self._map.grid[2, 0] = .3


		# Now that the map is updated, publish it!
		#rospy.loginfo("Scan is processed, publishing updated map.")
		self._map_data_pub.publish(grid_msg.info)
		self._map_pub.publish(grid_msg)



def main(args=None):
	rclpy.init(args=args)
	m = Mapper()
	rclpy.spin(m)


if __name__ == '__main__':
	main()






