import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao.lidar as Lidar
import lmao.util.pclmao as pclmao

from scipy.spatial.transform import Rotation as R

import time

# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# Import odometry message
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

# Tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



# Define the class for the raycast localization node
class RaycastLocalization(Node):
    def __init__(self, map, lidar, max_range, resolution, max_iterations, threshold):
        super().__init__('raycast_localization')
        self.map = map
        self.lidar = lidar
        self.max_range = max_range
        self.resolution = resolution
        self.max_iterations = max_iterations
        self.threshold = threshold
        points = []
        self.pos = np.array([0, 0, 0])
        self.orientation = np.array([0, 0, 0, 0])

        # Declare parameters
        from rcl_interfaces.msg import ParameterDescriptor
        self.declare_parameter('map_path', "map.ply", ParameterDescriptor("Path to the map file"))
        self.declare_parameter('lidar_topic', "/lidar", ParameterDescriptor("Topic to subscribe to for lidar data"))
        self.declare_parameter('max_range', 10, ParameterDescriptor("Maximum range of the lidar"))
        self.declare_parameter("world_frame", "odom", ParameterDescriptor("The world frame (origin of the map)"))
        self.declare_parameter("odom_topic", "/odom", ParameterDescriptor("Topic to publish odometry data to"))

        # Get parameters
        self.map_path = self.get_parameter("map_path").value
        self.lidar_topic = self.get_parameter("lidar_topic").value
        self.max_range = self.get_parameter("max_range").value
        self.world_frame = self.get_parameter("world_frame").value
        self.odom_topic = self.get_parameter("odom_topic").value

        # Create the subscriber
        self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)

        # Create publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Create tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        data = pclmao.extract_PointCloud2_data(msg)
        # x = data["x"], y = data["y"], z = data["z"]
        xyz = np.vstack((data["x"], data["y"], data["z"])).T
        xyz = xyz[np.logical_not(np.isnan(xyz).any(axis=1))]
        depth = data["range"]
        # Transform the data to the correct position
        try:
            t = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, msg.header.stamp)
        except TransformException as e:
            self.get_logger().error("Transform error: {}, when transforming from {} to {}".format(e, msg.header.frame_id, self.world_frame))
            return
        
        # extract the rotation and translation components of the transform
        rotation = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        
        # Construct the transformation matrix
        r = R.from_quat(rotation)
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = translation

        # Transform the data
        xyz = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
        xyz = np.dot(T, xyz.T).T
        xyz = xyz[:, :3]


        # Create new point cloud object for visualization in rviz
        pc = pclmao.construct_pointcloud2_msg({"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "range": depth})
        pc.header.frame_id = self.world_frame
        pc.header.stamp = msg.header.stamp
        self.lidar_pub.publish(pc)


    
    def load_map(self, map_path):
        self.map = o3d.io.read_triangle_mesh(map_path)

    def save_map(self, map_path):
        o3d.io.write_triangle_mesh(map_path, self.map)
