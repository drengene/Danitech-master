import numpy as np

test = np.array([[1,2,3],[5,4,3],[7,5,4]])

arr = np.array([3, 2, 1])

print(np.all(np.isin(test, arr), axis=1))