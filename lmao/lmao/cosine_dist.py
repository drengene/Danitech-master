import numpy as np
from sklearn.metrics.pairwise import paired_cosine_distances
import time

# Create dummy array of size [1000,10,3]
a = np.random.rand(1000,1000,3)
# Create dummy array of size [10,3]
b = np.random.rand(1000,3)

# Normalize all vectors in a
a = a / np.linalg.norm(a, axis=2)[:, :, np.newaxis]
# Normalize all vectors in b
b = b / np.linalg.norm(b, axis=1)[:, np.newaxis]


gt_time = time.time()
gt = np.zeros(a.shape[0])
# gt_cos = np.zeros((a.shape[0], b.shape[0]))
for i in range(a.shape[0]):
    gt[i] = np.mean( np.square(paired_cosine_distances(b, a[i])) )
    # gt_cos[i] = paired_cosine_distances(b, a[i]) 
gt_time = time.time() - gt_time
print("Ground truth time: ", gt_time)
print("Shape of gt: ", gt.shape)
# print(gt)
# print(gt_cos)


# cos_sim = 1-np.dot(ab, bc)/(np.linalg.norm(ab)*np.linalg.norm(bc)) 
# cos_sim_efficient = 1 - np.dot(ab, bc)

# Test subtract
# res = a-b.reshape(1,10,3)

# cos_sim_very_efficient = (np.linalg.norm(ab-bc)**2) *.5
ve_time = time.time()
ve = np.mean(
    np.square(
        0.5 * np.square(
            np.linalg.norm(
                a - b.reshape(1,b.shape[0],b.shape[1]), axis=2
                )
            )
        ), axis=1
    )
# ve_cos = 0.5 * np.square(
#             np.linalg.norm(
#                 a - b.reshape(1,10,3), axis=2
#                 )
#             )
ve_time = time.time() - ve_time
print("Very efficient time: ", ve_time)
print("Shape of ve: ", ve.shape)

print("ve is ", gt_time/ve_time, " times faster than gt")

# print(ve)
# print(ve_cos)

# print(cos_sim)
# print(cos_sim_efficient)
# print(cos_sim_very_efficient)