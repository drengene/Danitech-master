
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

# ros_dtype = [PointField.FLOAT32, PointField.FLOAT32, PointField.FLOAT32, PointField.FLOAT32, PointField.UINT32, PointField.UINT16, PointField.UINT8, PointField.UINT16, PointField.UINT32]
dtype = np.float32
itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

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
                    vertical_resolution=90/127,
                    rotation_rate=0,
                    high_lod=True,
                    yaw_offset=0.0,
                    enable_semantics=False
                )

        # asyncio.ensure_future(self.get_lidar_param())                        # Only ask for data after sweep is complete
        self.pub = self.create_publisher(PointCloud2, 'lidar_data', 10)

    def ros_pub(self):
        data_dict = {}
        pointcloud_data = self.lidarInterface.get_point_cloud_data(self.lidarPath).astype(np.float32)
        # intensity_data = self.lidarInterface.get_intensity_data(self.lidarPath).astype(np.float32)
        # t_data = np.zeros_like(intensity_data, dtype=np.uint32)
        # reflectivity_data = np.zeros_like(intensity_data, dtype=np.uint16)
        # ring_data = np.zeros_like(intensity_data, dtype=np.uint8)
        # ambient_data = np.zeros_like(intensity_data, dtype=np.uint16)
        range_data = self.lidarInterface.get_linear_depth_data(self.lidarPath) # Is type FLOAT32 in meters and should be converted to UNIT32 in mm
        range_data = range_data * 1000
        range_data = range_data.astype(np.uint32)

        # append all data to the dict
        data_dict['x'] = pointcloud_data[:,:,0]
        data_dict['y'] = pointcloud_data[:,:,1]
        data_dict['z'] = pointcloud_data[:,:,2]
        # data_dict['intensity'] = intensity_data
        # data_dict['t'] = t_data
        # data_dict['reflectivity'] = reflectivity_data
        # data_dict['ring'] = ring_data
        # data_dict['ambient'] = ambient_data
        data_dict['range'] = range_data
        


        self.construct_pointcloud2_msg(data_dict)


        # publish lidar data to ros as pointcloud2

        # print("publish lidar data")



        # This is a lot faster, but unsafe and untested, as it relies on the private _data attribute





    def construct_pointcloud2_msg(self, data_dict):
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
        sim_time = self.sim_context.current_time
        msg.header.stamp.sec = int(sim_time)
        msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        msg.header.frame_id = 'base_scan'
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
            byte_array = data_dict[field.name].reshape(-1).tobytes()

            data[:, field.offset:field.offset+datatype_size] = np.frombuffer(byte_array, dtype=np.uint8).reshape(-1, datatype_size)
            

        # Flatten data
        data = data.reshape(-1)

        msg._data = data

        self.pub.publish(msg)

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


    # def ros_pub_old(self):
    #         lidar_data = self.lidarInterface.get_point_cloud_data(self.lidarPath)
    #         # publish lidar data to ros as pointcloud2
    
    #         sim_time = self.sim_context.current_time
    #         self.msg.header.stamp.sec = int(sim_time)
    #         self.msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
    #         self.msg.header.frame_id = 'base_scan'
    #         self.msg.height = len(lidar_data[0])
    #         self.msg.width = len(lidar_data)

    #         self.msg.fields =  [PointField(
    #             name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
    #             for i, n in enumerate('xyz')]
    #         self.msg.is_bigendian = False
    #         self.msg.point_step = 12
    #         self.msg.row_step = self.msg.point_step * self.msg.width
    #         self.msg.is_dense = True
    #         #data = lidar_data.astype(dtype).tobytes() 
    #         #self.msg.data = data
    #         #self.msg.data = lidar_data.astype(dtype).tobytes() # This shit takes like 0.01 seconds
    #         self.msg._data = lidar_data.astype(dtype).tobytes() # This is a lot faster, but unsafe and untested, as it relies on the private _data attribute
    #         #print("time: ", time.time() - time_now)
    #         self.pub.publish(self.msg)
