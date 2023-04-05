import numpy as np

test = np.array([1,2,3])

arr = np.array([0, 0, 0])
arr = np.vstack([arr, test])

print(arr)
arr = np.vstack([arr, test])
print(arr)