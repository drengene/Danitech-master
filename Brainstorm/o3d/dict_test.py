from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np


test_dict = {
	'x': np.array([1, 2, 3]),
	'y': 2,
	'z': 3,
	'intensity': 4,
}

print(test_dict['x'].dtype)