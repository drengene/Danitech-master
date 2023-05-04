import numpy as np
from sklearn.metrics.pairwise import cosine_similarity
from sklearn.metrics.pairwise import cosine_distances
from sklearn.metrics.pairwise import paired_cosine_distances

correct = np.random.rand(100, 3)
other = np.random.rand(5, 100, 3)

print(correct.shape)

# for i in other:
# 	print(cosine(correct, i))

# Calculate cosine distance
cosine_sim = cosine_similarity(correct, other[0], dense_output=False)
print(cosine_sim.shape)


cosine_dist = paired_cosine_distances(correct, other[0])
print(cosine_dist.shape)
print(cosine_dist)
minval = np.argmin(cosine_dist)
print("Minimum distance: ", cosine_dist[minval], " at index: ", minval, " \nwith vector: ", other[0][minval], " and correct vector: ", correct[minval])