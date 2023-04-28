# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import ParameterDescriptor
import lmao_lib.util.pclmao as pclmao

# Python
import numpy as np

import cv2

cv2.namedWindow("Dir", cv2.WINDOW_NORMAL)
cv2.namedWindow("Generated dir", cv2.WINDOW_NORMAL)

class Analyse_Lidar(Node):
	def __init__(self):
		super().__init__("Analyse_Lidar")

		self.declare_parameter('lidar_topic', "/wagon/base_scan/lidar_data", ParameterDescriptor(description="Topic to subscribe to for lidar data"))
		self.lidar_topic = self.get_parameter("lidar_topic").value

		self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.lidar_topic))

	def convert_xyz_rpy(self, xyz):
		magnitudes = np.linalg.norm(xyz, axis=2)
		unit_vectors = np.divide(xyz, magnitudes[..., None])
		roll = np.arctan2(unit_vectors[..., 1], unit_vectors[..., 2])
		pitch = np.arctan2(-unit_vectors[..., 0], np.sqrt(unit_vectors[..., 1]**2 + unit_vectors[..., 2]**2))
		yaw = np.arctan2(unit_vectors[..., 0], unit_vectors[..., 1])

		rpy = np.stack((roll, pitch, yaw), axis=-1)

		print(rpy.shape)
		print(rpy[0, 0, :])
		# Show as image
		# Normalize length of vectors to 1
		rpy_image = np.zeros((128, 1024, 3), dtype=np.uint8)
		rpy_image[..., 0] = np.uint8((rpy[..., 0] + np.pi) / (2 * np.pi) * 255)
		rpy_image[..., 1] = np.uint8((rpy[..., 1] + np.pi) / (2 * np.pi) * 255)
		rpy_image[..., 2] = np.uint8((rpy[..., 2] + np.pi) / (2 * np.pi) * 255)
		return rpy_image

		
	def lidar_callback(self, msg):
		self.get_logger().info("Lidar data received")
		# Get from lidar data
		data = pclmao.extract_PointCloud2_data(msg)
		xyz = np.dstack((data['x'], data['y'], data['z']))
		xyz = np.rot90(xyz, 1, (1, 0))
		print(xyz.shape)

		# Get RPY for each point given origin as 0,0,0
		rpy_image = self.convert_xyz_rpy(xyz)
		cv2.imshow("Dir", rpy_image)

		rays = self.generate_rays()
		rays_image = self.convert_xyz_rpy(rays)
		cv2.imshow("Generated dir", rays_image)


		cv2.waitKey(1)

	def generate_rays(self, offset = 3*(np.pi/2)):
		rays = np.zeros((self.horizontal_lines,self.rays_per_line, 6), dtype=np.float32)
		#rays_optimized = np.zeros((self.horizontal_lines,self.rays_per_line, 6), dtype=np.float32)

		rot_x = np.sin(np.linspace(0 + offset, 2*np.pi + offset, self.rays_per_line , endpoint=False))
		rot_y = np.cos(np.linspace(0 + offset, 2*np.pi + offset, self.rays_per_line , endpoint=False))

		# dz = np.cos(np.linspace((np.pi)/4, 3*np.pi/4, horizontal_lines))
		dz = np.cos(np.linspace((np.pi)/2 - self.vertical_fov/2, (np.pi)/2 + self.vertical_fov/2, self.horizontal_lines, endpoint=True))
		dz_scale = np.sin(np.linspace((np.pi)/2 - self.vertical_fov/2, (np.pi)/2 + self.vertical_fov/2, self.horizontal_lines, endpoint=True))
		rays[:, :, 3] = rot_x * dz_scale[:, None]
		rays[:, :, 4] = rot_y * dz_scale[:, None]
		rays[:, :, 5] = dz[:, None]
		return rays

def main(args=None):
	rclpy.init(args=args)
	node = Analyse_Lidar()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()

