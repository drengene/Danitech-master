import numpy as np
import matplotlib.pyplot as plt
# Rotation
from scipy.spatial.transform import Rotation as R

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
from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication

# Load pickle file
QApp = QApplication([])
QWidget = QWidget()
# open dialog to select file


def get_odom_data(dict, dict_key):
	orientations = []
	positions = []
	velocities = []
	ang_velocities = []
	timesteps = []
	for msg in dict[dict_key]:
		orientations.append(R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
		positions.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
		velocities.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
		ang_velocities.append(np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]))
		timesteps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

	odom_dict = {"orientations" : orientations, "positions" : positions, "velocities" : velocities, "ang_velocities" : ang_velocities, "timesteps" : timesteps}
	return odom_dict

def get_twist_data(dict, dict_key):
	linear = []
	angular = []
	timesteps = []
	for msg in dict[dict_key]:
		linear.append(np.array([msg.linear.x, msg.linear.y, msg.linear.z]))
		angular.append(np.array([msg.angular.x, msg.angular.y, msg.angular.z]))
		timesteps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

	twist_dict = {"linear" : linear, "angular" : angular, "timesteps" : timesteps}
	return twist_dict

def get_joint_states(dict, dict_key):
	names = []
	positions = []
	velocities = []
	efforts = []
	timesteps = []
	for msg in dict[dict_key]:
		names.append(msg.name)
		positions.append(np.array(msg.position))
		velocities.append(np.array(msg.velocity))
		efforts.append(np.array(msg.effort))
		timesteps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

	joint_states_dict = {"name" : names, "positions" : positions, "velocities" : velocities, "efforts" : efforts, "timesteps" : timesteps}
	return joint_states_dict

def get_localization_data(file_path):

	with open(file_path, "rb") as f:
		data = pickle.load(f)

	positions = data[:][0]
	orientations = data[:][1]
	timesteps = data[:][2]


	localization_dict = {"orientations" : orientations, "positions" : positions, "timesteps" : timesteps}
	return localization_dict


def get_pose_array(dict, dict_key):
	plan = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in dict["dict_key"].poses])
	return plan



def main():
	file_path = QFileDialog.getOpenFileName(QWidget, 'Open file', '/home/daniel/Documents/master/rosbags/pose_data')[0]
	print(file_path)

	with open(file_path, "rb") as f:
		data = pickle.load(f)

	# Rear link pose

	rear_link_dict = get_odom_data(data, "rear_link_pose_gt")
	base_link_dict = get_odom_data(data, "base_link_pose_gt")
	waypoints = get_pose_array(data, "wayposes")
	plan = get_pose_array(data, "global_pln")
	cmd_vel = get_twist_data(data, "cmd_vel")
	joint_states_controller = get_joint_states(data, "joint_state_controller")
	joint_states = get_joint_states(data, "joint_states")
	odometry_ekf = get_odom_data(data, "odometry_ekf")
	


if __name__ == '__main__':
	main()