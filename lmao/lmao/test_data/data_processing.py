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
time_file = os.path.dirname(os.path.realpath(__file__)) + "/times.txt"

bag_file = QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")
print("Bag file: ", bag_file)
gt_positions, gt_orientations, gt_clock = load_from_bag(bag_file, '/wagon/base_link_pose_gt')

incs = [50, 100, 200, 400, 800, 1600, 3200]
dists = np.zeros((len(incs), len(incs)))
avg_dist = np.zeros((len(incs), len(incs)))
times = np.zeros((len(incs), len(incs)))

total_time = gt_clock[-1] - gt_clock[0]
print("Total time: ", total_time)


for pkl in pkl_files:
	# pkl filename is 50rays_100particles.pkl. Get the number of rays and particles from the filename
	n_rays = int(pkl.split("/")[-1].split("_")[0].split("rays")[0])
	n_particles = int(pkl.split("/")[-1].split("_")[1].split("particles")[0])
	#print("n_rays: ", n_rays, "n_particles: ", n_particles)
		
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
	ray_idx = incs.index(n_rays)
	particle_idx = incs.index(n_particles)
	dists[ray_idx, particle_idx] = rmse_dist
	avg_dist[ray_idx, particle_idx] = np.mean(dist)


# Load times from file 
with open(time_file, "r") as f:
	# Time is on format "Time for 50 rays and 50 particles: 22.71437668800354"
	for line in f:
		n_rays = int(line.split(" ")[2])
		n_particles = int(line.split(" ")[5])
		time = float(line.split(" ")[7])
		ray_idx = incs.index(n_rays)
		particle_idx = incs.index(n_particles)
		times[ray_idx, particle_idx] = time

plt.figure()
plt.imshow(dists, norm=LogNorm())
# plt.colorbar()
# Format colorbar as 4.123 instead of 10^4
plt.colorbar(format="%.3f", ticks=[0.1, 0.2, 0.5, 1, 2, 5, 10])
plt.xticks(np.arange(len(incs)), incs)
plt.yticks(np.arange(len(incs)), incs)
plt.xlabel("Particles")
plt.ylabel("Rays")
plt.title("RMSE of distance to ground truth")
plt.show()

plt.figure()
plt.imshow(avg_dist, norm=LogNorm())
plt.colorbar()
plt.xticks(np.arange(len(incs)), incs)
plt.yticks(np.arange(len(incs)), incs)
plt.xlabel("Particles")
plt.ylabel("Rays")
plt.title("Average distance to ground truth")
plt.show()

plt.figure()
plt.imshow(times/total_time, norm=LogNorm())
plt.colorbar()
plt.xticks(np.arange(len(incs)), incs)
plt.yticks(np.arange(len(incs)), incs)
plt.xlabel("Particles")
plt.ylabel("Rays")
plt.title("Computation time normalized to total time")
# Add value on top of each pixel
for i in range(len(incs)):
	for j in range(len(incs)):
		plt.text(j, i, round(times[i, j]/total_time, 2), ha="center", va="center", color="w")
plt.show()

# perf = np.divide(dists, times)
# plt.figure()
# plt.imshow(perf)
# plt.colorbar()
# plt.xticks(np.arange(len(incs)), incs)
# plt.yticks(np.arange(len(incs)), incs)
# plt.xlabel("Particles")
# plt.ylabel("Rays")
# plt.title("Performance")
# plt.show()
