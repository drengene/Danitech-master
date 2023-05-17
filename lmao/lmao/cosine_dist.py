import numpy as np
from sklearn.metrics.pairwise import paired_cosine_distances

ab = np.array([1, 1, 0])
bc = np.array([1, 1, 0])

output2 = paired_cosine_distances([ab], [bc])
print(output2)

cos_sim = 1-np.dot(ab, bc)/(np.linalg.norm(ab)*np.linalg.norm(bc)) 

print(cos_sim)