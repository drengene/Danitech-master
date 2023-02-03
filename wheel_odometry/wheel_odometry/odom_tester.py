# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
#Import twist
import time
from geometry_msgs.msg import Twist

class odom_tester(Node):
	def __init__(self):
		super().__init__('odom_tester')
		# Declare vehicle parameters
		from rcl_interfaces.msg import ParameterDescriptor
		self.declare_parameter('odom_topic', '/odometry/local', ParameterDescriptor(description="Wheel odometry topic name"))
		self.declare_parameter('cmd_vel_topic', '/cmd_vel', ParameterDescriptor(description="cmd vel topic"))
		
		# Get topic name parameters
		self.cmd_vel = self.get_parameter('cmd_vel_topic').value
		self.odom_topic = self.get_parameter('odom_topic').value

		# Create subscriber for joint state
		self.subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

		# Create publisher for wheel odometry
		self.publisher = self.create_publisher(Twist, self.cmd_vel, 10)

		


	def run(self):
		self.tester(1, 5)

		# Print values from odom_callback
		time.sleep(5)

		self.tester(1, 10)

		rclpy.spin_once(self)
		self.get_logger().info('Position x: %f' % self.pos_x)
		self.get_logger().info('Position y: %f' % self.pos_y)
		self.get_logger().info('Orientation z: %f' % self.theta)


		self.tester(-1, 10)

		time.sleep(5)
		rclpy.spin_once(self)
		

		# Print values from odom_callback
		self.get_logger().info('Position x: %f' % self.pos_x)
		self.get_logger().info('Position y: %f' % self.pos_y)
		self.get_logger().info('Orientation z: %f' % self.theta)

	def tester(self, direction, duration):
		self.init_time = time.time()

		print("starting to drive")
		while True:
			self.send_cmd_vel(1.0*direction, 0.7)
			#self.get_logger().info('Publishing cmd_vel: %f' % self.twist_msg.linear.x)
			time_now = time.time()
			if int(time_now) - int(self.init_time) > duration:
				break
			time.sleep(0.1)
		self.send_cmd_vel(0.0)


	def odom_callback(self, msg):
		print("received callback")
		self.pos_x = msg.pose.pose.position.x
		self.pos_y = msg.pose.pose.position.y
		self.theta = msg.pose.pose.orientation.z

	def send_cmd_vel(self, vel, ang_vel=0.0):
		self.twist_msg = Twist()
		self.twist_msg.linear.x = vel
		self.twist_msg.angular.z = ang_vel


		self.publisher.publish(self.twist_msg)


		
def main():
	rclpy.init()
	wheel_odom_node = odom_tester()
	wheel_odom_node.run()


	wheel_odom_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
