import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
# Rosbag
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

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
            msg = deserialize_cdr(rawdata, connection.msgtype)
            print(msg.header)

            # Now this data is a PointCloud2. Let's load it into a numpy array
            array_orig = np.frombuffer(msg.data, dtype=np.uint32)
            array_orig = array_orig.reshape(msg.height, msg.width, -1)
            # Discard other than x,y,z
            array = array_orig[:, :, 8].copy()
            #print(tiny_array.shape)
            # Convert to open3d
            #pcd = o3d.geometry.PointCloud()
            #pcd.points = o3d.utility.Vector3dVector(array.reshape(-1, 3))
            #o3d.visualization.draw_geometries([pcd])

            # Circularly shift every second row by 10 pixels
            array[0::4, :] = np.roll(array[0::4, :], -16, axis=1)
            array[1::4, :] = np.roll(array[1::4, :], -32, axis=1)
            array[2::4, :] = np.roll(array[2::4, :], -48, axis=1)
            array[3::4, :] = np.roll(array[3::4, :], -64, axis=1)

            fig, axs = plt.subplots(2, 1)
            axs[0].imshow(array_orig[:,:,8])
            axs[1].imshow(array)

            plt.show()



            exit()
            

if __name__ == '__main__':
    main()