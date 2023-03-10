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
                # dtype_list = [(field.name, np.dtype(self.datatype_dict[field.datatype])) for field in msg.fields]
                print(np.shape(msg.data))
                xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)[:,:,:3].copy()
                depth = np.frombuffer(msg.data, dtype=np.uint32).reshape(msg.height, msg.width, -1)[:,:,8].copy()
                #array = point_cloud.reshape(msg.height, msg.width, -1).copy()
                xyz[0::4, :] = np.roll(xyz[0::4, :], -16, axis=1)
                depth[0::4, :] = np.roll(depth[0::4, :], -16, axis=1)
                xyz[1::4, :] = np.roll(xyz[1::4, :], -32, axis=1)
                depth[1::4, :] = np.roll(depth[1::4, :], -32, axis=1)
                xyz[2::4, :] = np.roll(xyz[2::4, :], -48, axis=1)
                depth[2::4, :] = np.roll(depth[2::4, :], -48, axis=1)
                xyz[3::4, :] = np.roll(xyz[3::4, :], -64, axis=1)
                depth[3::4, :] = np.roll(depth[3::4, :], -64, axis=1)


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
