import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from time import time

if __name__ == "__main__":
    # Create meshes and convert to open3d.t.geometry.TriangleMesh .
    cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
    cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
    torus = o3d.geometry.TriangleMesh.create_torus().translate([0, 2, 2])
    torus = o3d.t.geometry.TriangleMesh.from_legacy(torus)
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate(
        [0, 0, 2])
    sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate(
        [1, 1, 3])

    # bunny = o3d.data.BunnyMesh()
    # mesh = o3d.io.read_triangle_mesh(bunny.path).translate([0, 0, 2])
    # mesh.compute_vertex_normals()
    # mesh.scale(20, center=mesh.get_center())
    # bunny = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    sphere = o3d.t.geometry.TriangleMesh.from_legacy(sphere)
    sphere2 = o3d.t.geometry.TriangleMesh.from_legacy(sphere2)

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(cube)
    scene.add_triangles(torus)
    scene.add_triangles(sphere2)
    #scene.add_triangles(bunny)
    _ = scene.add_triangles(sphere)

    camera_rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=[0, 0, 2],
        eye=[2, 3, 0],
        up=[0, 1, 0],
        width_px=640,
        height_px=480,
    )

    print(camera_rays.shape)
    print(camera_rays[0], camera_rays[1])




    # Create rays like os1-64.
    horizontal_lines = 64
    rays_per_line = 1024
    vertical_fov = np.pi / 2

    # os1-64 has 64 channels in the vertical axis and 1024 rays per channel.
    # The vertical FOV is 90 degrees, while the horizontal FOV is 360 degrees.
    # The rays are evenly distributed in the horizontal and vertical FOV.
    # The rays are on the form [x, y, z, dx, dy, dz][x, y, z, dx, dy, dz] ... , where the first three values are the origin of the ray, and the last three values are the direction of the ray.
    # First, create a grid of rays
    rays = np.zeros((horizontal_lines,rays_per_line, 6), dtype=np.float32)
    # Populate the directions with cos and sin of the angles.
    for i in range(horizontal_lines):
        # dz = np.cos(np.linspace((np.pi)/4, 3*np.pi/4, horizontal_lines))
        dz = np.cos(np.linspace((np.pi)/2 - vertical_fov/2, (np.pi)/2 + vertical_fov/2, horizontal_lines))
        # dz = np.cos(np.linspace((np.pi)/2 - np.pi/2, (np.pi)/2 + np.pi/2, 5))
        for j in range(rays_per_line):
            dx = np.sin(np.linspace(0, 2*np.pi, rays_per_line))
            dy = np.cos(np.linspace(0, 2*np.pi, rays_per_line))
            rays[i, j, :] = np.array([0, 0, 0, dx[j], dy[j], dz[i]])

    # ------- WARNING -------
    # This is wrong, because the rays have length > 1, where they should have length 1.
    # This is problematic because hit distances are calculated as the length of the ray.   
    # I can't figure out how to normalize the rays, so I'm just going to use the camera rays for now. 
        
    print(rays.shape)
    print(rays[0], rays[1])
    # We can directly pass the rays tensor to the cast_rays function.
    # Time raycasting.
    t0 = time()
    ans = scene.cast_rays(rays)
    print(f"Time: {time() - t0:.3f} s")



    # # Test 1000 times and get the average time.
    # t0 = time()
    # for _ in range(1000):
    #     ans = scene.cast_rays(rays)
    # t1 = time()

    # print(f"Time: {(t1 - t0) / 1000:.3f} s")
    # print("Giving a framerate of ", 1 / ((t1 - t0) / 1000), "fps")

    # Visualize the results.
    plt.imshow(ans['t_hit'].numpy())
    plt.show()
    plt.imshow(np.abs(ans['primitive_normals'].numpy()))
    plt.show()
    plt.imshow(np.abs(ans['geometry_ids'].numpy()), vmax=3)
    plt.show()