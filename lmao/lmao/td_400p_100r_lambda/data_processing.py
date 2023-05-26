import pickle
import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d

# Load all pkl files in current directory
import glob
import os

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication
QApp = QApplication([])
QWidget = QWidget()

from scipy.spatial.transform import Rotation as R

from matplotlib.colors import LogNorm

import matplotlib.patches as mpatches
from matplotlib.ticker import FormatStrFormatter, NullFormatter

def get_localization_data(file_path):

	with open(file_path, "rb") as f:
		data = pickle.load(f)

	orientations = []
	positions = []
	timesteps = []
	for msg in data:
		positions.append(msg[0])
		orientations.append(msg[1])
		timesteps.append(msg[2])


	localization_dict = {"orientations" : orientations, "positions" : positions, "timesteps" : timesteps}
	return localization_dict

def create_lineset(positions, color):
	lines = []
	for i in range(len(positions)-1):
		lines.append([i, i+1])
	lineset = o3d.geometry.LineSet()
	lineset.points = o3d.utility.Vector3dVector(positions)
	lineset.lines = o3d.utility.Vector2iVector(lines)
	lineset.colors = o3d.utility.Vector3dVector([color for i in range(len(lines))])
	return lineset



def load_from_bag(bag_file, topic):
	gt_positions = []
	gt_orientations = []
	clock = []
	with Reader(bag_file) as reader:
		#for connection in reader.connections:
			#print(connection.topic, connection.msgtype)
		for connection, timestamp, rawdata in reader.messages():
			if connection.topic == topic:
				msg = deserialize_cdr(rawdata, connection.msgtype)
				#print(msg.header.frame_id)
				#print(msg.pose.pose.position.x)
				gt_positions.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
				gt_orientations.append([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
				clock.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

	return gt_positions, gt_orientations, clock


# Get all pkl files in directory of this file
pkl_files = glob.glob(os.path.dirname(os.path.realpath(__file__)) + "/*.pkl")
print(pkl_files)

# time_file = os.path.dirname(os.path.realpath(__file__)) + "/times.txt"

bag_file = QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")
print("Bag file: ", bag_file)
gt_positions, gt_orientations, gt_clock = load_from_bag(bag_file, '/wagon/base_link_pose_gt')

# Sort pkl files alphabetically
pkl_files.sort()

n_tests = []
n_lambdas = []
for pkl in pkl_files:
	n_tests.append(int(pkl.split("/")[-1].split("_")[1].split("test")[0]))
	n_lambdas.append(float(pkl.split("/")[-1].split("_")[0].split("lambda")[0]))

n_tests = np.max(n_tests) # Number of tests per lambda
n_lambdas = np.max(n_lambdas) # Number of lambdas tested

# Create dictionary for storing data
rmse_dict = {}
avg_dist_dict = {}

for pkl in pkl_files:
	# pkl filename is 0.5lambda_2test.pkl. Get lambda and n_test
	n_lambda = float(pkl.split("/")[-1].split("_")[0].split("lambda")[0])
	n_test = int(pkl.split("/")[-1].split("_")[1].split("test")[0])
	print("n_lambda: ", n_lambda)
	print("n_test: ", n_test)
		
	localization_dict = get_localization_data(pkl)

	gt_positions = np.array(gt_positions)
	loc_positions = np.array(localization_dict["positions"])

	dist = []
	angular_difference = []

	for i, pos in enumerate(loc_positions):
		idx = np.argmin(np.linalg.norm(gt_positions - pos, axis=1))
		dist.append(np.linalg.norm(gt_positions[idx] - pos))
		# Calculate difference in roll, pitch and yaw between gt and localization
		gt_rpy = R.from_quat(gt_orientations[idx]).as_euler('xyz', degrees=False)
		loc_rpy = R.from_quat(localization_dict["orientations"][i]).as_euler('xyz', degrees=False)
		# Get as three separate angles, respecting circularity
		roll_diff = np.arctan2(np.sin(gt_rpy[0] - loc_rpy[0]), np.cos(gt_rpy[0] - loc_rpy[0]))
		pitch_diff = np.arctan2(np.sin(gt_rpy[1] - loc_rpy[1]), np.cos(gt_rpy[1] - loc_rpy[1]))
		yaw_diff = np.arctan2(np.sin(gt_rpy[2] - loc_rpy[2]), np.cos(gt_rpy[2] - loc_rpy[2]))
		angular_difference.append([roll_diff, pitch_diff, yaw_diff])
	
	# Calculate RMSE of dist
	dist = np.array(dist)
	rmse_dist = np.sqrt(np.mean(np.square(dist)))
	#print("RMSE with " + str(n_rays) + " rays and " + str(n_particles) + " particles: ", rmse_dist)
	# Place in correct position in dist
	# Place data in dictionary
	if n_lambda not in rmse_dict:
		rmse_dict[n_lambda] = []
		avg_dist_dict[n_lambda] = []
	rmse_dict[n_lambda].append(rmse_dist)
	avg_dist_dict[n_lambda].append(np.mean(dist))

# Print dicts formatted such that a keys values are printed on the same line
print("RMSE: ")
for key, value in rmse_dict.items():
	print(key, " ", value)
	print("Mean: ", np.mean(value))
	print("Std: ", np.std(value))
print("Average distance: ")

for key, value in avg_dist_dict.items():
	print(key, " ", value)
	print("Mean: ", np.mean(value))
	print("Std: ", np.std(value))



imgsize = (5, 5)

# Extract the parameter values and RMSE lists from the dictionary
parameters = list(rmse_dict.keys())
rmse_values = list(rmse_dict.values())
# Create a figure and axis
fig, ax = plt.subplots()
# Create the boxplot
boxplot = ax.boxplot(rmse_values, patch_artist=True, notch=True)
# Add labels and title
# X label is greek letter lambda
ax.set_xlabel(r'$\lambda$')
ax.set_ylabel('RMSE')
ax.set_title('RMSE Box Plot (' + str(n_tests+1) + ' tests)')
# Set x-axis tick labels
ax.set_xticklabels(parameters)
# Customize the colors of the boxplot elements
colors = ['lightblue', 'lightgreen', 'lightpink', 'lightyellow', 'lightgrey']
for patch, color in zip(boxplot['boxes'], colors):
    patch.set_facecolor(color)
#Make the y-axis logarithmic
ax.set_yscale('log')
# Add gridlines
ax.yaxis.grid(True)
# Set size
fig.set_size_inches(imgsize)
# Create lower margin to keep x-labels visible
plt.subplots_adjust(bottom=0.2)
# Remove the white space from the right of the plot
plt.subplots_adjust(right=0.98)
# Set y ticks
ax.yaxis.set_minor_formatter(NullFormatter())
ax.yaxis.set_major_formatter(FormatStrFormatter("%1.1f"))
ax.set_yticks([0.1, 0.2, 0.5, 1, 2, 5, 10])

# save the figure
fig.savefig(os.path.dirname(os.path.realpath(__file__)) + "/Localizer_lambda_RMSE_boxplot.png")
# Show the plot
plt.show()

# A good caption for this plot is:
# "Average distance between localization and ground truth for different values of lambda"


# Plot average distance
avg_dist_values = list(avg_dist_dict.values())
# Create a figure and axis
fig, ax = plt.subplots()
# Create the boxplot
boxplot = ax.boxplot(avg_dist_values, patch_artist=True, notch=True)
# Add labels and title
ax.set_xlabel(r'$\lambda$')
ax.set_ylabel('Average distance')
# Title should convey the number of tests
ax.set_title('Average distance Box Plot (' + str(n_tests+1) + ' tests)')
# Set x-axis tick labels
ax.set_xticklabels(parameters)
# Customize the colors of the boxplot elements
colors = ['lightblue', 'lightgreen', 'lightpink', 'lightyellow', 'lightgrey']
for patch, color in zip(boxplot['boxes'], colors):
	patch.set_facecolor(color)
#Make the y-axis logarithmic
ax.set_yscale('log')
# Add gridlines
ax.yaxis.grid(True)
# Set size
fig.set_size_inches(imgsize)
# Create lower margin to keep x-labels visible
plt.subplots_adjust(bottom=0.2)
# Remove the white space at the right of the plot
plt.subplots_adjust(right=0.98)
# Set y ticks
ax.yaxis.set_minor_formatter(NullFormatter())
ax.yaxis.set_major_formatter(FormatStrFormatter("%1.1f"))
ax.set_yticks([0.1, 0.2, 0.5, 1, 2, 5, 10])
# save the figure
fig.savefig(os.path.dirname(os.path.realpath(__file__)) + "/Localizer_lambda_avg_dist_boxplot.png")
# Show the plot
plt.show()

# A good caption for this plot is:
# "RMSE between localization and ground truth for different values of lambda"