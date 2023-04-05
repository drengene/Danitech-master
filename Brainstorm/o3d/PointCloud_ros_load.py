import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# Rosbag
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
#import lmao.lidar as Lidar
import time

DATATYPE_SIZE = {
	PointField.INT8: 1,
	PointField.UINT8: 1,
	PointField.INT16: 2,
	PointField.UINT16: 2,
	PointField.INT32: 4,
	PointField.UINT32: 4,
	PointField.FLOAT32: 4,
	PointField.FLOAT64: 8,
}

DATATYPE_NPDTYPE = {
	PointField.INT8: np.int8,
	PointField.UINT8: np.uint8,
	PointField.INT16: np.int16,
	PointField.UINT16: np.uint16,
	PointField.INT32: np.int32,
	PointField.UINT32: np.uint32,
	PointField.FLOAT32: np.float32,
	PointField.FLOAT64: np.float64,
}

NPDTYPE_DATATYPE = {
    np.dtype('int8') : PointField.INT8,
	np.dtype('uint8') : PointField.UINT8,
	np.dtype('int16') : PointField.INT16,
	np.dtype('uint16') : PointField.UINT16,
	np.dtype('int32') : PointField.INT32,
	np.dtype('uint32') : PointField.UINT32,
	np.dtype('float32') : PointField.FLOAT32,
	np.dtype('float64') : PointField.FLOAT64,
}

class LoadRosbag:
	def __init__(self, bagpath, topics = []):
		self.bagpath = bagpath
		self.topics = topics
		self.messages = []
		self.connections = []
		with Reader(self.bagpath) as reader:
			for connection in reader.connections:
				if connection.topic in self.topics:
					self.connections.append(connection)   

	

	def get_messages(self):
		with Reader(self.bagpath) as reader:
			for connection in reader.connections:
				print(connection.topic, connection.msgtype)
			# iterate over messages
			for connection, timestamp, rawdata in reader.messages():
				if connection.msgtype == "sensor_msgs/msg/PointCloud2":
					msg = deserialize_cdr(rawdata, connection.msgtype)
					yield extract_PointCloud2_data(msg)


def extract_PointCloud2_data(msg, visualize=False):
	data = np.array(msg.data)

	# Reshape data such that every row is a point
	data = data.reshape(-1, msg.point_step)


	data_dict = {}

	for field in msg.fields:
		# Get datatype size
		datatype_size = DATATYPE_SIZE[field.datatype]
		datatype_npdtype = DATATYPE_NPDTYPE[field.datatype]
		# Extract data
		data_dict[field.name] = np.frombuffer(data[:, field.offset:field.offset+datatype_size].copy(), dtype=datatype_npdtype).reshape(msg.height, msg.width, -1)
	
	#lidar = Lidar.Lidar("/home/junge/master_ws/src/Danitech-master/Brainstorm/o3d/2023-03-09-11-59-56_OS-0-128-992037000169-1024x10.json")
	#lidar.unstagger(data_dict)

	if visualize:
		# Plot range
		plt.imshow(data_dict['range'])
		plt.show()

		# Plot xyz
		fig, axs = plt.subplots(3, 1)
		axs[0].imshow(data_dict['x'])
		axs[1].imshow(data_dict['y'])
		axs[2].imshow(data_dict['z'])
		plt.show()

	return data_dict


def construct_pointcloud2_msg(data_dict):
	t0 = time.time()
	# Test if data_dict is valid dict
	if not isinstance(data_dict, dict):
		raise Exception("data_dict is not a dict")

	# Construct PointCloud2 message
	msg = PointCloud2()
	
	offset = 0
	for key in data_dict:
		field = PointField()
		field.name = key
		field.datatype = NPDTYPE_DATATYPE[data_dict[key].dtype]
		field.offset = offset
		field.count = 1
		offset += DATATYPE_SIZE[field.datatype]
		msg.fields.append(field)

	if len(msg.fields) == 0:
		raise Exception("data_dict does not contain any fields")
	
	msg.height = data_dict[msg.fields[0].name].shape[0]
	msg.width = data_dict[msg.fields[0].name].shape[1]
	msg.point_step = offset
	msg.row_step = msg.point_step * msg.width
	msg.is_bigendian = False
	msg.is_dense = True

	# Construct data
	data = np.zeros((msg.height*msg.width, msg.point_step), dtype=np.uint8)


	for field in msg.fields:
		datatype_size = DATATYPE_SIZE[field.datatype]
		#byte_array = data_dict[field.name].reshape(-1).tobytes()
		data[:, field.offset:field.offset+datatype_size] = np.frombuffer(data_dict[field.name], dtype=np.uint8).reshape(-1, datatype_size)
		#data[:, field.offset:field.offset+datatype_size] = np.frombuffer(byte_array, dtype=np.uint8).reshape(-1, datatype_size)
	# Flatten data
	data = data.reshape(-1)
	
	
	msg._data = data
	t1 = time.time()
	print("Time to construct message: ", t1-t0)

	return msg


def callback(msg):
	print("Received message")
	extract_PointCloud2_data(msg, visualize=True)



def main(args=None):
	rclpy.init(args=args)

	node = rclpy.create_node('PointCloud_ros_load')
	pub = node.create_publisher(PointCloud2, "/os1_cloud_node/points", 10)
	sub = node.create_subscription(PointCloud2, "/os1_cloud_node/points", callback, 10)

	# Load rosbag	
	bagpath = '/home/junge/Documents/rosbag2_2023_03_09-15_53_39'
	topics = ["/os1_cloud_node/points"]
	loader = LoadRosbag(bagpath, topics)

	# Get messages
	msg = loader.get_messages()
	msg1 = msg.__next__()
	# print(msg1)
	#print(msg.__next__())

	# Construct PointCloud2 message
	msg2 = construct_pointcloud2_msg(msg1)

	for i in range(1,2):
		# Create publisher
		
		# Publish message
		node.get_logger().info('Publishing PointCloud2 message')
		pub.publish(msg2)
		rclpy.spin_once(node)
		time.sleep(1)





	# Visualize
	#extract_PointCloud2_data(msg2, visualize=True)

	





	
			

if __name__ == '__main__':
	main()