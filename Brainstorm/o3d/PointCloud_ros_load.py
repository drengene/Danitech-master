import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# Rosbag
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

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

    def get_messages(self, topic):
        for connection, timestamp, rawdata in self.connections:
            if connection.topic == topic:
                msg = deserialize_cdr(rawdata, connection.msgtype)
                yield msg


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('PointCloud_ros_load')

    # Read rosbag at path = /home/junge/Documents/rosbag2_2023_03_09-13_28_13/rosbag2_2023_03_09-13_28_13_0.db3
    with Reader('/home/junge/Documents/rosbag2_2023_03_09-15_53_39') as reader:
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        # iterate over messages
        for connection, timestamp, rawdata in reader.messages():
            if connection.msgtype == "sensor_msgs/msg/PointCloud2":
                msg = deserialize_cdr(rawdata, connection.msgtype)
                print(msg.point_step)
                print(msg.data.shape[0])

                data = msg.data
                print(msg.fields)

                # Reshape data such that every row is a point
                data = data.reshape(-1, msg.point_step)
                print("Shape of data: ", data.shape)
                data_dict = {}

                for field in msg.fields:
                    data_dict[field.name] = 1
                    # Get datatype size
                    datatype_size = DATATYPE_SIZE[field.datatype]
                    datatype_npdtype = DATATYPE_NPDTYPE[field.datatype]
                    # Extract data
                    data_dict[field.name] = np.frombuffer(data[:, field.offset:field.offset+datatype_size].copy(), dtype=datatype_npdtype).reshape(msg.height, msg.width, -1)
                    # Print
                    print(field.name, field.offset, field.datatype, datatype_size)

                print(data_dict.keys())

                plt.imshow(data_dict['range'])
                plt.show()

                # Plot xyz
                fig, axs = plt.subplots(3, 1)
                axs[0].imshow(data_dict['x'])
                axs[1].imshow(data_dict['y'])
                axs[2].imshow(data_dict['z'])
                plt.show()
            

if __name__ == '__main__':
    main()