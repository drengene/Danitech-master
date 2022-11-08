
import omni                                                     # Provides the core omniverse apis
import asyncio                                                  # Used to run sample asynchronously to not block rendering thread
from omni.isaac.range_sensor import _range_sensor               # Imports the python bindings to interact with lidar sensor
from pxr import UsdGeom, Gf, UsdPhysics                         # pxr usd imports used to create the cube
from omni.kit.commands import execute

import numpy as np

import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import PointCloud2, PointField

ros_dtype = PointField.FLOAT32
dtype = np.float32
itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.



class lidar(Node):
    def __init__(self, lidarName, lidarParent="/wagon"):
        super().__init__('lidar_publisher', namespace=lidarParent)
        self.stage = omni.usd.get_context().get_stage()                      # Used to access Geometry
        self.sim_context = omni.isaac.core.SimulationContext()      # Used to interact with simulation
        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface() # Used to interact with the LIDAR

        # These commands are the Python-equivalent of the first half of this tutorial
        self.lidarName = lidarName
        self.lidarParent = lidarParent
        self.lidarPath = lidarParent + lidarName
        result, prim = omni.kit.commands.execute(
                    "RangeSensorCreateLidar",
                    path=self.lidarName,
                    parent=self.lidarParent,
                    min_range=0.4,
                    max_range=100.0,
                    draw_points=False,
                    draw_lines=False,
                    horizontal_fov=360.0,
                    vertical_fov=90.0,
                    horizontal_resolution=360/1024,
                    vertical_resolution=90/64,
                    rotation_rate=10.0,
                    high_lod=True,
                    yaw_offset=0.0,
                    enable_semantics=False
                )

        # asyncio.ensure_future(self.get_lidar_param())                        # Only ask for data after sweep is complete
        self.pub = self.create_publisher(PointCloud2, 'lidar_data', 10)
        self.msg = PointCloud2()


    def ros_pub(self):
        lidar_data = self.lidarInterface.get_point_cloud_data(self.lidarPath)
        # publish lidar data to ros as pointcloud2
  
        sim_time = self.sim_context.current_time
        self.msg.header.stamp.sec = int(sim_time)
        self.msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.msg.header.frame_id = 'base_scan'
        self.msg.height = len(lidar_data[0])
        self.msg.width = len(lidar_data)

        self.msg.fields =  [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        self.msg.is_bigendian = False
        self.msg.point_step = 12
        self.msg.row_step = self.msg.point_step * self.msg.width
        self.msg.is_dense = True
        #data = lidar_data.astype(dtype).tobytes() 
        #self.msg.data = data
        self.msg.data = lidar_data.astype(dtype).tobytes() # This shit takes like 0.01 seconds
        #print("time: ", time.time() - time_now)
        self.pub.publish(self.msg)




    async def get_lidar_param(self):                                    # Function to retrieve data from the LIDAR
        await omni.kit.app.get_app().next_update_async()            # wait one frame for data
        #self.timeline.pause()                                            # Pause the simulation to populate the LIDAR's depth buffers
        depth = self.lidarInterface.get_linear_depth_data(self.lidarParent+self.lidarPath)
        zenith = self.lidarInterface.get_zenith_data(self.lidarParent+self.lidarPath)
        azimuth = self.lidarInterface.get_azimuth_data(self.lidarParent+self.lidarPath)
        pointcloud = self.lidarInterface.get_point_cloud_data(self.lidarParent+self.lidarPath)
        semantics = self.lidarInterface.get_semantic_data(self.lidarParent+self.lidarPath)
        print("depth", depth)                                       # Print the data
        print("zenith", zenith)
        print("azimuth", azimuth)
        print("pointcloud", pointcloud)
        print("semantics", semantics)

