from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2

import numpy as np
import struct

class BagLoader:
	def __init__(self, bagpath, topics = []):
		self.bagpath = bagpath
		self.topics = topics
		self.messages = []
		self.connections = []
		self.reader = Reader(self.bagpath)
		self.datatype_dict = {
			PointField.INT8: 'int8',
			PointField.UINT8: 'uint8',
			PointField.INT16: 'int16',
			PointField.UINT16: 'uint16',
			PointField.INT32: 'int32',
			PointField.UINT32: 'uint32',
			PointField.FLOAT32: 'float32',
			PointField.FLOAT64: 'float64'
		}
		#with Reader(self.bagpath) as reader:
		#    for connection in reader.connections:
		#        if connection.topic in self.topics:
		#            self.connections.append(connection)        
		self.pixel_shift_1024x10 = [64,43,23,3,63,43,23,4,62,42,23,4,61,42,24,5,60,42,24,6,59,41,24,6,59,41,24,7,58,41,24,7,58,41,24,8,57,41,24,8,57,41,24,8,57,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,8,56,40,24,7,56,40,23,7,56,40,23,6,57,40,23,6,57,40,23,5,57,40,23,5,58,40,22,4,58,40,22,3,59,40,22,2,60,41,21,1,61,41,21,0]

	def get_messages(self, topic):
		for connection, timestamp, rawdata in self.connections:
			if connection.topic == topic:
				msg = deserialize_cdr(rawdata, connection.msgtype)
				yield msg

	def get_pointcloud(self, topic):
		
		self.reader.open()
		#dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('t', np.uint32), ('reflectivity', np.uint16) ('ring', np.uint8), ('ambient', np.uint16), ('range', np.uint32)])
		for connection, timestamp, rawdata in self.reader.messages():
			if connection.topic == topic:
				msg = deserialize_cdr(rawdata, connection.msgtype)
				point_step = msg.point_step
				row_step = msg.row_step
				# dtype_list = [(field.name, np.dtype(self.datatype_dict[field.datatype])) for field in msg.fields]
				print(np.shape(msg.data.reshape(msg.height, msg.width, -1)))
				
				xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)[:,:,:3].copy()
				depth = np.frombuffer(msg.data, dtype=np.uint32).reshape(msg.height, msg.width, -1)[:,:,8].copy()

				print

				# shift every row with the pixel shift
				for i in range(0, 128):
					xyz[i, :] = np.roll(xyz[i, :], self.pixel_shift_1024x10[i], axis=0)
					depth[i, :] = np.roll(depth[i, :], self.pixel_shift_1024x10[i], axis=0)




				#array = point_cloud.reshape(msg.height, msg.width, -1).copy()
				# roll_factor = 17 # The image loads in kinda weird, so we need to roll it a bit
				# xyz[0::4, :] = np.roll(xyz[0::4, :], -int(roll_factor), axis=1)
				# depth[0::4, :] = np.roll(depth[0::4, :], -roll_factor, axis=1)
				# xyz[1::4, :] = np.roll(xyz[1::4, :], -int(2*roll_factor), axis=1)
				# depth[1::4, :] = np.roll(depth[1::4, :], -int(2*roll_factor), axis=1)
				# xyz[2::4, :] = np.roll(xyz[2::4, :], -int(3*roll_factor), axis=1)
				# depth[2::4, :] = np.roll(depth[2::4, :], -int(3*roll_factor), axis=1)
				# xyz[3::4, :] = np.roll(xyz[3::4, :], -int(4*roll_factor), axis=1)
				# depth[3::4, :] = np.roll(depth[3::4, :], -int(4*roll_factor), axis=1)


				self.reader.close() # Probably not the correct place to close the reader
				# print("Field names: ", msg.fields)
				return xyz, depth
			
	def get_sim_pointcloud(self, topic):
		
		self.reader.open()
		#dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('t', np.uint32), ('reflectivity', np.uint16) ('ring', np.uint8), ('ambient', np.uint16), ('range', np.uint32)])
		for connection, timestamp, rawdata in self.reader.messages():
			if connection.topic == topic:
				msg = deserialize_cdr(rawdata, connection.msgtype)
				# dtype_list = [(field.name, np.dtype(self.datatype_dict[field.datatype])) for field in msg.fields]
				print(np.shape(msg.data.reshape(msg.height, msg.width, -1)))
				print("height: ", msg.height)
				print("width: ", msg.width)
				xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)[:,:,:3].copy()
				depth = np.frombuffer(msg.data, dtype=np.uint32).reshape(msg.height, msg.width, -1)[:,:,8].copy()
				#array = point_cloud.reshape(msg.height, msg.width, -1).copy()
				roll_factor = 17 # The image loads in kinda weird, so we need to roll it a bit
				xyz[0::4, :] = np.roll(xyz[0::4, :], -int(roll_factor), axis=1)
				depth[0::4, :] = np.roll(depth[0::4, :], -roll_factor, axis=1)
				xyz[1::4, :] = np.roll(xyz[1::4, :], -int(2*roll_factor), axis=1)
				depth[1::4, :] = np.roll(depth[1::4, :], -int(2*roll_factor), axis=1)
				xyz[2::4, :] = np.roll(xyz[2::4, :], -int(3*roll_factor), axis=1)
				depth[2::4, :] = np.roll(depth[2::4, :], -int(3*roll_factor), axis=1)
				xyz[3::4, :] = np.roll(xyz[3::4, :], -int(4*roll_factor), axis=1)
				depth[3::4, :] = np.roll(depth[3::4, :], -int(4*roll_factor), axis=1)


				self.reader.close() # Probably not the correct place to close the reader
				# print("Field names: ", msg.fields)
				return xyz, depth
			

	def get_pointcloud2(self, topic):
		
		self.reader.open()
		#dtypes = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32), ('t', np.uint32), ('reflectivity', np.uint16) ('ring', np.uint8), ('ambient', np.uint16), ('range', np.uint32)])
		for connection, timestamp, rawdata in self.reader.messages():
			if connection.topic == topic:
				msg = deserialize_cdr(rawdata, connection.msgtype)
				# dtype_list = [(field.name, np.dtype(self.datatype_dict[field.datatype])) for field in msg.fields]
				print(np.shape(msg.data))
				point_cloud_x = np.frombuffer(msg.data[np.arange(0, msg.data.shape[0], 36)[:4]], dtype=np.float32)
				point_cloud_y = np.frombuffer(msg.data, dtype=np.float32, offset=4, count=1)
				point_cloud_z = np.frombuffer(msg.data, dtype=np.float32, offset=8, count=1)
				point_cloud_D = np.frombuffer(msg.data, dtype=np.uint32, offset=32, count=1)
				# print all shapes
				print(np.shape(point_cloud_x))
				print(np.shape(point_cloud_y))
				print(np.shape(point_cloud_z))
				print(np.shape(point_cloud_D))

				# xyz = np.vstack((point_cloud_x, point_cloud_y, point_cloud_z)).T

				array = xyz.reshape(msg.height, msg.width, -1).copy()
				# array[0::4, :] = np.roll(array[0::4, :], -16, axis=1)
				# array[1::4, :] = np.roll(array[1::4, :], -32, axis=1)
				# array[2::4, :] = np.roll(array[2::4, :], -48, axis=1)
				# array[3::4, :] = np.roll(array[3::4, :], -64, axis=1)


				self.reader.close() # Probably not the correct place to close the reader
				# print("Field names: ", msg.fields)
				return array
