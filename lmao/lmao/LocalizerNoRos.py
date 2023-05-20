import rclpy


from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import time

import cv2

from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication
n_Rays = [50, 100, 200, 400, 800, 1600, 3200]
n_Particles = [50, 100, 200, 400, 800, 1600, 3200]
test_file_loc = "/home/danitech/master_ws/src/Danitech-master/lmao/lmao/test_data/"
test_file_loc = "/home/junge/master_ws/src/Danitech-master/lmao/lmao/test_data/"


import logging


def main(args=None):
	rclpy.init(args=args)
	#QApp = QApplication([])
	#QWidget = QWidget()
	for i in n_Rays:
		for j in n_Particles:

			logging.basicConfig(filename=(test_file_loc + str(i) + "rays_" + str(j) + "particles.log"), level=logging.DEBUG)
			logger = logging.getLogger(__name__)

			from Localizer import Localizer

			try:
				
				time.sleep(2)

				print("Starting test with " + str(i) + " rays and " + str(j) + " particles")

				localizer = Localizer(ros=False, n_rays=i, n_particles=j, fname=(test_file_loc + str(i) + "rays_" + str(j) + "particles.pkl"), nname="n"+str(i) + "rays_" + str(j) + "particles")
				bag_file = "/home/danitech/Documents/bags/island_boy_to_rule_them_all" #QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")
				bag_file = "/home/junge/Documents/bags/island_boy_to_rule_them_all"
				time0 = 0

				t0 = time.time()

				with Reader(bag_file) as reader:
					print("We in da bag")
					#for connection in reader.connections:
						#print(connection.topic, connection.msgtype)
					for connection, timestamp, rawdata in reader.messages():
						if connection.topic == "/wagon/base_scan/lidar_data":
							msg = deserialize_cdr(rawdata, connection.msgtype)
							localizer.lidar_callback(msg)
							#localizer.viz_loop()
						elif connection.topic == "/wagon/base_link_odom_gt":
							msg = deserialize_cdr(rawdata, connection.msgtype)

							time1 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
							if time1 - time0 > 1:
								print(time1)
								time0 = time1
							localizer.odom_callback(msg)
						elif connection.topic == "/cmd_vel":
							msg = deserialize_cdr(rawdata, connection.msgtype)
							if localizer.cmd_vel_callback(msg) == False:
								break
				print("loop finished")
				# Write time to file
				t1 = time.time()
				with open(test_file_loc + "times.txt", "a") as f:
					f.write("Time for " + str(i) + " rays and " + str(j) + " particles: " + str(t1-t0) + "\n")
				print("Continuing")
				# destroy all cv2 windows
				cv2.destroyAllWindows()
			except Exception as e:
				logger.exception(e)
				print(e)
				continue


if __name__ == "__main__":
	main()