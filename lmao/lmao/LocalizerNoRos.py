import rclpy

from Localizer import Localizer
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import time

from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication
n_Rays = [50, 100, 200, 400, 800, 1600, 3200].reverse()
n_Particles = [50, 100, 200, 400, 800, 1600, 3200].reverse()
test_file_loc = "/home/junge/master_ws/src/Danitech-master/lmao/lmao/test_data/"

import logging


def main(args=None):
	rclpy.init(args=args)
	#QApp = QApplication([])
	#QWidget = QWidget()
	for i in n_Rays:
		for j in n_Particles:

			logging.basicConfig(filename=(test_file_loc + i + "rays_" + j + "particles.log"), level=logging.DEBUG)
			logger = logging.getLogger(__name__)

			try:

				localizer = Localizer(ros=False, n_rays=i, n_particles=j, fname=(test_file_loc + i + "rays_" + j + "particles.pkl"))
				bag_file = "/home/junge/Documents/bags/island_boy_to_rule_them_all" #QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")

				time = 0

				t0 = time.time()

				with Reader(bag_file) as reader:
					print("We in da bag")
					#for connection in reader.connections:
						#print(connection.topic, connection.msgtype)
					for connection, timestamp, rawdata in reader.messages():
						if connection.topic == "/wagon/base_scan/lidar_data":
							msg = deserialize_cdr(rawdata, connection.msgtype)
							localizer.lidar_callback(msg)
							localizer.viz_loop()
						elif connection.topic == "/wagon/base_link_odom_gt":
							msg = deserialize_cdr(rawdata, connection.msgtype)

							time1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
							if time1 - time > 1:
								print(time1)
								time = time1
							localizer.odom_callback(msg)
						elif connection.topic == "/cmd_vel":
							msg = deserialize_cdr(rawdata, connection.msgtype)
							if localizer.cmd_vel_callback(msg) == False:
								break
				# Write time to file
				t1 = time.time()
				with open(test_file_loc + "times.txt", "a") as f:
					f.write("Time for " + i + " rays and " + j + " particles: " + str(t1-t0) + "\n")
			except Exception as e:
				logger.exception(e)
				print(e)
				continue


if __name__ == "__main__":
	main()