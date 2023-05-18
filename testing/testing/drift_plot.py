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


file_path = QFileDialog.getOpenFileName(QWidget, 'Open file', '/home/daniel/Documents/master/rosbags/pose_data')[0]
print(file_path)

with open(file_path, "rb") as f:
	data = pickle.load(f)

# Rear link pose
rear_orientations = []
rear_positions = []
rear_velocities = []
rear_ang_velocities = []
rear_timesteps = []
for msg in data["rear_link_pose_gt"]:
	rear_orientations.append(R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
	rear_positions.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
	rear_velocities.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
	rear_ang_velocities.append(np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]))
	rear_timesteps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)



# Base link pose
base_orientations = []
base_positions = []
base_velocities = []
base_ang_velocities = []
base_timesteps = []
for msg in data["base_link_pose_gt"]:
	base_orientations.append(R.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))
	base_positions.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]))
	base_velocities.append(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]))
	base_ang_velocities.append(np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]))
	base_timesteps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

rear_directions = np.zeros((len(rear_orientations), 3))
base_directions = np.zeros((len(base_orientations), 3))
for i, o in enumerate(rear_orientations):
	rear_directions[i] = o.apply([1, 0, 0])
for i, o in enumerate(base_orientations):
	base_directions[i] = o.apply([1, 0, 0])

rear_heading_angles = np.zeros(len(rear_orientations))
base_heading_angles = np.zeros(len(base_orientations))
for i in range(len(rear_orientations)):
	# If velocity is less than 0.2 m/s and twist.z is less than 0.1 rad/s, set heading angle to 0 and continue
	if np.linalg.norm(rear_velocities[i]) < 0.2 and np.linalg.norm(rear_ang_velocities[i]) < 0.1:
		rear_heading_angles[i] = 0
		continue
	# Get angle between rear_directions[i] and rear_velocities[i]
	rear_heading_angles[i] = np.arccos(np.dot(rear_directions[i], rear_velocities[i]) / (np.linalg.norm(rear_directions[i]) * np.linalg.norm(rear_velocities[i])))
for i in range(len(base_orientations)):
	# If velocity is less than 0.2 m/s and twist.z is less than 0.1 rad/s, set heading angle to 0 and continue
	if np.linalg.norm(base_velocities[i]) < 0.2 and np.linalg.norm(base_ang_velocities[i]) < 0.1:
		base_heading_angles[i] = 0
		continue
	# Get angle between base_directions[i] and base_velocities[i]
	base_heading_angles[i] = np.arccos(np.dot(base_directions[i], base_velocities[i]) / (np.linalg.norm(base_directions[i]) * np.linalg.norm(base_velocities[i])))

cmd_vels = np.zeros((len(data["cmd_vel"]), 2))
# cmd_timesteps = data["cmd_vel_time"]
for i, msg in enumerate(data["cmd_vel"]):
	cmd_vels[i] = np.array([msg.linear.x, msg.angular.z])

# Plot heading angles over time
plt.plot(rear_timesteps, rear_heading_angles)
plt.plot(base_timesteps, base_heading_angles)

# plt.plot(cmd_timesteps, cmd_vels[:, 1])
# Add legend
plt.legend(["Rear link", "Base link", "Angular kink command"])
# Add labels
plt.xlabel("Time (s)")
plt.ylabel("Heading angle (rad)")
plt.show()


# Convert to numpy arrays


