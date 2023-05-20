import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node


# Odometry
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

# Tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import pickle
import glob
import os



class Record_to_pickle(Node):
	def __init__(self):
		super().__init__("record_to_pickle")
		
		sub1 = self.create_subscription(Odometry, "/wagon/rear_link_pose_gt", self.odom_callback_0, 10)
		sub2 = self.create_subscription(Odometry, "/wagon/base_link_pose_gt", self.odom_callback_1, 10)
		sub3 = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

		self.most_recent_timestamp = 0

		# Create data dict with lists for each topic
		self.data_dict = {
			"rear_link_pose_gt": [],
			"base_link_pose_gt": [],
			"cmd_vel": [],
			"cmd_vel_time": []
		}

	def callback(self, msg, topic):
		# add data to dict
		self.data_dict[topic].append(msg)
		if topic == "cmd_vel":
			self.data_dict["cmd_vel_time"].append(self.most_recent_timestamp)
		else:
			self.most_recent_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

	def odom_callback_0(self, msg):
		self.callback(msg, "rear_link_pose_gt")

	def odom_callback_1(self, msg):
		self.callback(msg, "base_link_pose_gt")

	def cmd_vel_callback(self, msg):
		self.callback(msg, "cmd_vel")


def main(args=None):
	rclpy.init(args=args)
	node = Record_to_pickle()

	# Spin until ctrl + c
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	print("Pickle with", len(node.data_dict), "topics saved")
	print("Topics:")
	for topic in node.data_dict:
		print(topic)
		print("With length:", len(node.data_dict[topic]))
	# Save data to pickle file at file location
	file_location = os.path.dirname(os.path.realpath(__file__))
	file_name = "data_low_fric.pickle"
	file_path = os.path.join(file_location, file_name)
	with open(file_path, "wb") as f:
		pickle.dump(node.data_dict, f)
	print("Data saved to pickle file")
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()	
