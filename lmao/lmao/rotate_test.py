import numpy as np
# Scipy
from scipy.spatial.transform import Rotation as R
import functools
import time

# Create 800 random rotations
r = R.random(800)

# Create x, y, 3 vector
np.random.seed(0)
v_ogg = np.random.rand(1000, 3) # 5 particles, 10 rays, 3 dimensions
v = np.zeros((800, 1000, 3))
v2 = np.zeros((800, 1000, 3))

t0 = time.time()
for i, r_ in enumerate(r):
    v[i] = r_.apply(v_ogg[i])
print(v.shape)
print("With for loop: ", time.time() - t0)

# Apply all rotations to the v_ogg with map
t0 = time.time()
v2 = map(lambda r_: r_.apply(v_ogg), r)
v2 = np.array(list(v2))
print(v2.shape)
print("With map: ", time.time() - t0)

# Test if v and v2 are equal
print(np.allclose(v, v2))

