import numpy as np
import struct

pointcloud_data = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
intensity_data = np.array([10.0, 20.0, 30.0])
t_data = np.array([1, 2, 3], dtype=np.uint32)
reflectivity_data = np.array([100, 200, 300], dtype=np.uint16)
ring_data = np.array([1, 2, 3], dtype=np.uint8)
ambient_data = np.array([1000, 2000, 3000], dtype=np.uint16)
range_data = np.array([1.23, 4.56, 7.89], dtype=np.float32)
range_data = range_data * 1000
range_data = range_data.astype(np.uint32)

# Create a structured array with one element per data point
dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32),         ('t', np.uint32), ('reflectivity', np.uint16), ('ring', np.uint8),         ('ambient', np.uint16), ('range', np.uint32)]
structured_data = np.empty(len(pointcloud_data), dtype=dtype)
structured_data['x'] = pointcloud_data[:, 0]
structured_data['y'] = pointcloud_data[:, 1]
structured_data['z'] = pointcloud_data[:, 2]
structured_data['intensity'] = intensity_data
structured_data['t'] = t_data
structured_data['reflectivity'] = reflectivity_data
structured_data['ring'] = ring_data
structured_data['ambient'] = ambient_data
structured_data['range'] = range_data

# Encode each data point as a byte string
byte_array = b''
for data_point in structured_data.tolist():
    byte_string = b''
    for value in data_point:
        if isinstance(value, float):
            byte_string += struct.pack('f', value)
        
            print("f", "handling type:", value)
            

        elif isinstance(value, np.uint32):
            byte_string += struct.pack('I', value)
            print("I, handling type:", value.dtype)

        elif isinstance(value, np.uint16):
            byte_string += struct.pack('H', value)
            print("H, handling type:", value.dtype)

        elif isinstance(value, np.uint8):
            byte_string += struct.pack('B', value)
            print("B, handling type:", value.dtype)

    byte_array += byte_string

print(byte_array)