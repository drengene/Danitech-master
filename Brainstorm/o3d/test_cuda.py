import open3d as o3d

# Check avaliability of cuda
print(o3d.core.cuda.is_available())