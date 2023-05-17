import numpy as np
from scipy.spatial.transform import Rotation as R


def get_lidar_rays(particles, rotations, rays):
    new_rays = np.zeros((particles.shape[0], 6))
    #new_directions = self.rotations.apply(rays)
    # Rotate the ray directions to the particles
    new_rays[:, :3] = rotations.apply(rays[:, :3])
    # Create a new origo for each ray, using the particles as origo
    new_origo = np.repeat(particles[:, :3], rays.shape[0], axis=0)
    # Add the new origo to the new directions
    new_rays[:, 3:] = new_origo
    new_rays[:, :3] += new_origo
    return new_rays


a = np.array([[1, 2, 3, 4, 5, 6], [4, 5, 6, 7, 8, 9], [7, 8, 9, 10, 11, 12]])
n = 4
# I want the result to be: [[1,2,3], [1,2,3], [1,2,3], [4,5,6], [4,5,6], [4,5,6], [7,8,9], [7,8,9], [7,8,9]] for n = 3

particles = np.random.uniform(0, 10, (100, 3))

yaw = np.random.uniform(0, 2*np.pi, particles.shape[0])
pitch = np.random.normal(0, np.pi/20, particles.shape[0])
roll = np.random.normal(0, np.pi/20, particles.shape[0])

# Convert the particles to quaternions
quats = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T).as_quat()
rotations = R.from_quat(quats)

print("Particles shape: {}".format(particles.shape))
print("Rotations shape: {}".format(rotations.as_quat().shape))
print("A shape: {}".format(a.shape))
new_rays = get_lidar_rays(particles, rotations, a)
print("New rays shape: {}".format(new_rays.shape))
