import math

from geometry_msgs.msg import TransformStamped, PoseStamped
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster



def quaternion_from_euler(ai, aj, ak):
	ai /= 2.0
	aj /= 2.0
	ak /= 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q


class FramePublisher(Node):

	def __init__(self):
		super().__init__('wagon_tf2_frame_publisher')

		# Declare and acquire `turtlename` parameter
		self.wagon_name = self.declare_parameter(
		'base_link_name', 'base_link').get_parameter_value().string_value

		# Initialize the transform broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)
		self.secs = 0
		self.nanosecs = 0
		# Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
		# callback function on each message
		self.pose_subscription = self.create_subscription(
			Odometry,
			'/wagon/' + self.wagon_name + '_pose_gt',
			self.handle_wagon_pose,
			1)
		self.pose_subscription  # prevent unused variable warning
		self.get_logger().info('Subscribed to /wagon/' + self.wagon_name + '_pose_gt')

		# self.clock_sub = self.create_subscription(
		# 	Clock,
		# 	'/clock',
		# 	self.handle_clock,
		# 	1)
			
		# self.clock_sub  # prevent unused variable warning
	

	# def handle_clock(self, msg):
	# 	self.secs = msg.clock.sec
	# 	self.nanosecs = msg.clock.nanosec
	# 	# print("Time: " + str(self.time))


	def handle_wagon_pose(self, msg):
		t = TransformStamped()

		# Read message content and assign it to
		# corresponding tf variables
		t.header.stamp.sec = msg.header.stamp.sec
		t.header.stamp.nanosec = msg.header.stamp.nanosec
		t.header.frame_id = 'base_link'
		t.child_frame_id = 'world'

		rotation = R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		translation = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
		
		# Create the transformation matrix
		transform = np.eye(4)
		transform[:3, :3] = rotation.as_matrix()
		transform[:3, 3] = translation

		# Compute the inverse transformation matrix
		inverse_transform = np.linalg.inv(transform)

		# Extract the inverse translation and rotation values
		inverse_translation = inverse_transform[:3, 3]
		inverse_rotation = R.from_matrix(inverse_transform[:3, :3])

		# Print the results
		#print("Inverse Translation: ", inverse_translation)
		#print("Inverse Rotation: ", inverse_rotation.as_euler('xyz', degrees=True))

		# Assign the translation and rotation to the
		# transform message
		t.transform.translation.x = inverse_translation[0]
		t.transform.translation.y = inverse_translation[1]
		t.transform.translation.z = inverse_translation[2]
		t.transform.rotation.x = inverse_rotation.as_quat()[0]
		t.transform.rotation.y = inverse_rotation.as_quat()[1]
		t.transform.rotation.z = inverse_rotation.as_quat()[2]
		t.transform.rotation.w = inverse_rotation.as_quat()[3]
		
		# Send the transformation
		self.tf_broadcaster.sendTransform(t)


def main():
	rclpy.init()
	node = FramePublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass

	rclpy.shutdown()