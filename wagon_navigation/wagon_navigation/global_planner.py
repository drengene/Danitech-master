import open3d as o3d
import numpy as np
import sys
import heapq
import time
import copy
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry


class global_planner():
    def __init__(self, file_path, load_valid_points_from_path=False):
        #super().__init__('global_planner')
        # rclpy.init()

        self.load_ply_file(file_path)
        self.convert_to_tensor()
        #pose_subscriber = self.create_subscription(Odometry, "/wagon/base_link_pose_gt", self.pose_callback, 10)

        #self.adjacency_list(self.mesh_map)

        normal_mesh = self.get_normals(self.mesh_map)


        self.adj_list = self.adjacency_list(self.mesh_map)

        if load_valid_points_from_path:
           self.load_valid_verticies(load_valid_points_from_path)
        else:
           self.determine_valid_vertices(0.85, 1.5, "/home/danitech/Documents/maps/island_boy_")
                
        # self.valid_points = np.ones(self.verticies.shape[0])

        # self.valid_points[self.vertex_normals[:, 2] < 0.8] = 0

        self.start_stop_chooser(self.mesh_map)
        #self.destroy_subscription(pose_subscriber)
        #self.points = np.array([50970, 558829])

        path = None

        for waypoint in range(self.points.shape[0]-1):
            if path is None:
                path = self.a_star(self.adj_list, self.points[waypoint], self.points[waypoint +1 ])
            else:
                path = np.concatenate((path, self.a_star(self.adj_list, self.points[waypoint], self.points[waypoint +1])[1:]))

        # path = self.a_star(self.adj_list, self.points[0], self.points[1]) # start at 7000
        if path is None:
            print("No path found")
            sys.exit()
        
        self.color_path(normal_mesh, path)
        
        self.global_waypoints = self.convert_path(path)

    def get_global_waypoints(self):
        return self.global_waypoints

    def load_ply_file(self, file_path):
        self.mesh_map = o3d.io.read_triangle_mesh(file_path)

    def convert_to_tensor(self):
        self.mesh_map_t = o3d.t.geometry.TriangleMesh.from_legacy(self.mesh_map)
        # print(self.mesh_map_tensor)

    def get_normals(self, mesh):

        print("Computing normals and rendering it ...")
        mesh.compute_vertex_normals()
        # print("normals: \n", np.asarray(mesh.triangle_normals))
        self.triangle_normals = np.asarray(mesh.triangle_normals)

        self.verticies = np.asarray(mesh.vertices)
        self.vertex_normals = np.asarray(mesh.vertex_normals)
        #points = self.verticies[self.path[1:-1]]
        # o3d.visualization.draw_geometries([mesh])

        # self.pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        return mesh
        


    def adjacency_list(self, mesh):

        mesh.compute_adjacency_list()

        # convert to the adjacency list to a numpy array of arrays, instead of a list of sets
        adj_list = np.asarray(mesh.adjacency_list)


        for i in range(len(adj_list)):
            adj_list[i] = np.asarray(list(adj_list[i]))

        # print("adjacency list: \n", adj_list) 
        # print("adjacency list index: \n", adj_list[1][1])

        return adj_list



    # def path_to_lineset(self, path):
    def load_valid_verticies(self, file_path="/home/daniel/Documents/master/valid_points.npy"):
        # check if path exists and then load it
        self.valid_points = np.load(file_path)
        print("valid points loaded from file: ", np.count_nonzero(self.valid_points))


    def determine_valid_vertices(self, normal_z_threshold, invalidation_radius=1, path="/home/daniel/Documents/master/"):
        self.valid_points = np.ones(self.verticies.shape[0])
        self.visited_points = np.ones(self.verticies.shape[0]) * -1

        self.valid_points[self.vertex_normals[:, 2] < normal_z_threshold] = 0
        print("valid points: ", np.count_nonzero(self.valid_points))
        if np.count_nonzero(self.valid_points) == 0:
            exit("No valid points found, try lowering the normal_z_threshold or choose correct ply file")
        invalid_points = np.where(self.valid_points == 0)[0] # create list of "hard" invalid point indexes
        print("invalid points: ", invalid_points.shape)
        sys.setrecursionlimit(4000)

        # Can possibly be optimized by only looking at invalid points that have valid points as neighbors
        for point_idx in invalid_points:
            # print("invalid point: ", point_idx)
            self.recursive_i = 0
            self.invalidate_in_radius(self.adj_list, invalidation_radius, point_idx, point_idx) # point_idx is given twice, since we have to compare the distance from the oringal invalid point
            #print("recursive calls: ", self.recursive_i)

        print("valid points after recursion: ", np.count_nonzero(self.valid_points))

        # Save the valid points to a npy file
        filename = "valid_points" + "_" + str(normal_z_threshold) + "_" + str(invalidation_radius) + ".npy"
        print("saving points to filename", path, filename)
        np.save(path + filename, self.valid_points)


    def invalidate_in_radius(self, adj_list, radius, point_idx, neighbor_idx):
        self.recursive_i += 1
        if self.visited_points[neighbor_idx] == point_idx:
            return
        
        self.visited_points[neighbor_idx] = point_idx


        dist = np.linalg.norm(self.verticies[neighbor_idx] - self.verticies[point_idx])
        # print("vertxe val_n", self.verticies[neighbor_idx], "neighbor_idx", neighbor_idx)
        # print("vertxe val_p", self.verticies[point_idx], "point_idx", point_idx)

        if dist > radius:
            return
        
        self.valid_points[neighbor_idx] = 0


        for neighbor in adj_list[neighbor_idx]:
            self.invalidate_in_radius(adj_list, radius, point_idx, neighbor)
        




    def determine_valid_vertices_(self, normal_z_threshold, invalidation_radius=1, path="/home/daniel/Documents/master/"):
        self.valid_points = np.ones(self.verticies.shape[0])
        self.visited_points = np.ones(self.verticies.shape[0]) * -1

        self.valid_points[self.vertex_normals[:, 2] < normal_z_threshold] = 0
        print("valid points: ", np.count_nonzero(self.valid_points))
        if np.count_nonzero(self.valid_points) == 0:
            exit("No valid points found, try lowering the normal_z_threshold or choose correct ply file")
        invalid_points = self.prep_world_for_random(self.mesh_map, normal_z_threshold)
        print("invalid points: ", invalid_points.shape)
        sys.setrecursionlimit(4000)

        # Can possibly be optimized by only looking at invalid points that have valid points as neighbors
        for point_idx in invalid_points:
            # print("invalid point: ", point_idx)
            self.recursive_i = 0
            self.invalidate_in_radius_(self.adj_list, invalidation_radius, point_idx, point_idx) # point_idx is given twice, since we have to compare the distance from the oringal invalid point
            #print("recursive calls: ", self.recursive_i)

        print("valid points after recursion: ", np.count_nonzero(self.valid_points))

        # Save the valid points to a npy file
        filename = "valid_points" + "_" + str(normal_z_threshold) + "_" + str(invalidation_radius) + ".npy"
        print("saving points to filename", path, filename)
        np.save(path + filename, self.valid_points)


    def invalidate_in_radius_(self, adj_list, radius, point_idx, neighbor_idx):
        self.recursive_i += 1
        if self.visited_points[neighbor_idx] == point_idx:
            return
        
        self.visited_points[neighbor_idx] = point_idx


        dist = np.linalg.norm(self.verticies[neighbor_idx] - self.verticies[point_idx])
        # print("vertxe val_n", self.verticies[neighbor_idx], "neighbor_idx", neighbor_idx)
        # print("vertxe val_p", self.verticies[point_idx], "point_idx", point_idx)

        if dist > radius:
            return
        
        self.valid_points[neighbor_idx] = 0


        for neighbor in adj_list[neighbor_idx]:
            self.invalidate_in_radius_(adj_list, radius, point_idx, neighbor)
        

    def prep_world_for_random(self, world, normal_z_threshold=0.8):
        print("World has normals: ", world.has_triangle_normals(), "\nWorld has vertex normals: ", world.has_vertex_normals())
        # Check if world triangles has normals
        if not world.has_triangle_normals():
            print("Computing normals")
            # Compute normals
            world.compute_triangle_normals()
        else:
            print("World has normals")
        
        # Remove all triangles with normal z < 0.8
        print("Removing triangles with normal z < 0.8")
        world.remove_triangles_by_mask(np.asarray(world.triangle_normals)[:, 2] < normal_z_threshold)
        world.remove_unreferenced_vertices()
        world = world.remove_degenerate_triangles()

        # Find all non manifold edges and show them in red
        non_manifold_edges_indicis = world.get_non_manifold_edges(allow_boundary_edges = False)
        
        # non_manifold_edges_indicies is open3d.cuda.pybind.utility.Vector2iVector
        non_manifold_indices = np.array(non_manifold_edges_indicis).ravel()
        
        print("Shape of non manifold indices: ", non_manifold_indices.shape)
        non_manifold_indices = np.unique(non_manifold_indices)
        print("Shape of non manifold indices after unique: ", non_manifold_indices.shape)

        return non_manifold_indices

    def color_path(self, triangle_mesh, path):
        # Get the vertex and face arrays from the triangle mesh
        # print("path: \n", path)
        # print("path normals", self.triangle_normals[path[1:-1]])
        # print("path vertices", self.vertex_normals[path[1:-1]])
        vertices = np.asarray(triangle_mesh.vertices)
        vertices = vertices.tolist()
        faces = triangle_mesh.triangles
        
        # Create a new LineSet object to visualize the path
        lines = []
        colors =[]
        points =[]
        for i in range(0, len(path)-1, 1):
            # Get the start and end vertices for the current edge
            start_vertex = int(path[i])
            end_vertex = int(path[i+1])
            start_point = vertices[start_vertex]
            end_point = vertices[end_vertex]

            #points.append(start_point)
             
            # Add the edge to the LineSet
            lines.append([start_vertex, end_vertex])
            
            # Set the color of the edge to red
            # colors.append([1, 0, 0])


        # print("triangle normals:", np.asarray(triangle_mesh.triangle_normals)[2])
        # print("vertex normals:" , np.asarray(triangle_mesh.vertex_normals)[2])
        normals = np.asanyarray(triangle_mesh.triangle_normals)
        normals[normals[:, 2] < 0.8] = 0

        triangle_mesh.triangle_normals = o3d.utility.Vector3dVector(normals)
        # triangle_mesh.vertex_normals = normals


        colors = [[1, 0, 0] for i in range(len(lines))]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        # print(np.asarray(line_set.lines))
        
        # Visualize the mesh and path together
        o3d.visualization.draw_geometries([triangle_mesh, line_set], point_show_normal=True)



    def start_stop_chooser(self, mesh):
        print("Select start and stop points by holding shift, then clicking on the desired points")

        normals = np.asanyarray(mesh.triangle_normals)
        normals[normals[:, 2] < 0.9] = 0
     
        mesh.triangle_normals = o3d.utility.Vector3dVector(normals)

        
        # Color all the points in the mesh that are not in self.valid_points
        colors = np.asarray(mesh.vertex_colors)
        

        index = np.where(self.valid_points == 1)[0]

        print("coloring invalid points")        
        # colors[index] = [0, 0, normal_z_color]
        # colors[self.valid_points == 1] = [0, 0, self.vertex_normals[x][2]]

        # colors[self.valid_points == 0] = [0.1, 0.1, 0]

        colors[:] = [0.5, 0.5, 0.5]
        colors[index] = self.vertex_normals[index] * [0.7, 0.7, 0.2] + 0.5

        # for i in range(colors.shape[0]):
        #     if i in index:
        #         colors[i] = self.vertex_normals[i] * [0.7, 0.7, 0.2] + 0.5

        #     else:
        #         colors[i] = [0.5, 0.5, 0.5]
        # colors[self.valid_points == 1] = [0, 0, self.vertex_normals[:][2]]
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)


        self.points = np.array([], dtype=int)
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(mesh)

        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(np.array([[0,0,0]]))
        pcd.paint_uniform_color([0, 0, 1])

        self.vis.add_geometry(pcd)
        self.vis.register_selection_changed_callback(self.start_stop_handler)
        self.vis.run()  # user picks points

    def start_stop_handler(self):
        print(self.vis.get_render_option())
        picked_points= self.vis.get_picked_points()
        print("picked points: ", picked_points[0].index)
        if (self.valid_points[picked_points[0].index] == 0):
            print("Invalid point chosen. Please choose a new point further from a wall.")

        else:
            print("valid point chosen")
            self.points = np.append(self.points, int(picked_points[0].index))
            print("Current points: ", self.points)
        
        # if len(self.points) == 2:
        #     self.vis.destroy_window()


    def pose_callback(self, msg):
        self.base_pose = msg.pose.pose


    def check_neighbor_feasibility(self, adj_list, vertex_index, neighbor_index):
        # check if the neighbor is a wall
        # normals_diff = abs(self.vertex_normals[vertex_index][2] - self.vertex_normals[neighbor_index][2])



        #if self.vertex_normals[neighbor_index][2] > 0.8 and normals_diff < 0.2:
        if self.valid_points[neighbor_index] == 1:
            return True
        else:
            return False
            

    def find_triangle_index(self, adj_list, vertex_index, neighbor_index):
        # check for any shared adjecent verticies in the adj_list
        common_verticies = np.intersect1d(adj_list[vertex_index], adj_list[neighbor_index])
        
        mesh_triangles = np.asarray(self.mesh_map.triangles)
        # Find the triangle that contains the common verticies

        tri_verts1 = np.array([vertex_index, neighbor_index, common_verticies[0]])
        tri_verts2 = np.array([vertex_index, neighbor_index, common_verticies[1]])


        common_tri_list = np.all(np.isin(mesh_triangles, tri_verts1), axis=1)
        common_tri_list2 = np.all(np.isin(mesh_triangles, tri_verts2), axis=1)


        tri_loc = np.array([np.where(common_tri_list == True)[0][0], np.where(common_tri_list2 == True)[0][0]])
        # print(tri_loc)
        # print(mesh_triangles[tri_loc])
        # print("Triangle normals", self.triangle_normals[tri_loc][:,2])

        if np.all(self.triangle_normals[tri_loc][:,2]) > 0.8:
            print(self.vertex_normals[neighbor_index][2])


    # def a_star(self, adj_list, start_index, goal_index):
    #     frontier = [(0, start_index, None)]
    #     explored = set()
    #     g_scores = {start_index: 0}
        
    #     def reconstruct_path(current_node):
    #         path = [current_node[1]]
    #         while current_node[2]:
    #             current_node = current_node[2]
    #             path.append(current_node[1])
    #         return list(reversed(path))
        
    #     while frontier:
    #         current_node = heapq.heappop(frontier)
    #         if current_node[1] == goal_index:
    #             return reconstruct_path(current_node)
            
    #         explored.add(current_node[1])
            
    #         for neighbor_index in adj_list[current_node[1]]:
    #             if neighbor_index in explored:
    #                 continue
                
    #             if not self.check_neighbor_feasibility(adj_list, current_node[1], neighbor_index):
    #                 continue

    #             cost_to_neighbor = g_scores[current_node[1]] + self.heuristic_cost_estimate(current_node[1], neighbor_index)
    #             if neighbor_index not in g_scores or cost_to_neighbor < g_scores[neighbor_index]:
    #                 g_scores[neighbor_index] = cost_to_neighbor
    #                 heuristic_to_goal = self.heuristic_cost_estimate(neighbor_index, goal_index)
    #                 f_score = cost_to_neighbor + heuristic_to_goal
    #                 neighbor_node = (f_score, neighbor_index, current_node)
    #                 heapq.heappush(frontier, neighbor_node)
        
    #     return None

    # def heuristic_cost_estimate(self, start_idx, end_idx):

    #     start = self.verticies[start_idx]
    #     end = self.verticies[end_idx]
    #     return np.linalg.norm(start-end)
    
    def a_star(self, adj_list, start_index, goal_index):
        frontier = [(0, start_index, None)]
        explored = set()
        g_scores = {start_index: 0}
        
        def reconstruct_path(current_node):
            path = [current_node[1]]
            while current_node[2]:
                current_node = current_node[2]
                path.append(current_node[1])
            return list(reversed(path))
        
        while frontier:
            current_node = heapq.heappop(frontier)
            if current_node[1] == goal_index:
                return reconstruct_path(current_node)
            
            explored.add(current_node[1])
            
            for neighbor_index in adj_list[current_node[1]]:
                if neighbor_index in explored:
                    continue
                
                if not self.check_neighbor_feasibility(adj_list, current_node[1], neighbor_index):
                    continue

                cost_to_neighbor = g_scores[current_node[1]] + self.heuristic_cost_estimate(current_node[1], neighbor_index, current_node[2])
                if neighbor_index not in g_scores or cost_to_neighbor < g_scores[neighbor_index]:
                    g_scores[neighbor_index] = cost_to_neighbor
                    heuristic_to_goal = self.heuristic_cost_estimate(neighbor_index, goal_index)
                    f_score = cost_to_neighbor + heuristic_to_goal
                    neighbor_node = (f_score, neighbor_index, current_node)
                    heapq.heappush(frontier, neighbor_node)
        
        return None

    def heuristic_cost_estimate(self, start_idx, end_idx, prev_idx=None):
        if prev_idx:
            a = self.verticies[prev_idx[1]]
            b = self.verticies[start_idx]
            c = self.verticies[end_idx]
            ab = b-a
            bc = c-b
            # Find cosine similarity between the two vectors
            cos_sim = np.dot(ab, bc)/(np.linalg.norm(ab)*np.linalg.norm(bc)) 
            angle_price = 2 - cos_sim

        start = self.verticies[start_idx]
        end = self.verticies[end_idx]
        if prev_idx:
            return np.linalg.norm(start-end) * angle_price
        else:
            return np.linalg.norm(start-end)
    
    def convert_path(self, path, seperation=2):

        
        # create global path only using points every 2 meters
        global_path = [self.verticies[path[0]]]
        for i in range(1, len(path)-1):
            start = global_path[-1]
            end = self.verticies[path[i]]
            dist = np.linalg.norm(start-end)
            if dist > seperation:
                global_path.append(end)
                print('appended point', end)
        global_path = np.array(global_path)
        # global_path = self.verticies[path]
        print(global_path)


        return global_path

    def send_to_local_planner(self):
        # Send the path to the local planner

        pass


class ros_planner(Node):
    def __init__(self, plan):
        super().__init__('ros_planner')
        self.path_pub = self.create_publisher(PoseArray, 'global_plan', 10)
        #self.clock_sub = self.create_subscription(Clock, 'clock', self.clock_callback, 10)
        #rclpy.spin_once(Node)
        # self.path_publisher(plan)
        while True:
            self.path_publisher(plan)
            time.sleep(1)
            self.get_logger().info('Publishing path')


    def clock_callback(self, msg):
        self.secs = msg.clock.sec
        self.nanosecs = msg.clock.nanosec
        self.time = self.secs + self.nanosecs * 1e-9

    def path_publisher(self, plan):
        path = PoseArray()
        path.header.frame_id = "world"
        path.header.stamp = self.get_clock().now().to_msg()
        # path.header.stamp.sec = self.secs
        # path.header.stamp.nanosec = self.nanosecs

        for i in range(len(plan)):
            pose = Pose()
            pose.position.x = plan[i][0]
            pose.position.y = plan[i][1]
            pose.position.z = plan[i][2]
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)
        # Add empty 



def main():
    # global_planner("/home/daniel/Documents/master/isaac_map.ply", "/home/daniel/Documents/master/valid_points.npy")
    # planner =  global_planner("/home/daniel/Documents/master/maps/easter_island_boy.ply", "/home/daniel/Documents/master/maps/island_boy_valid_points_0.85_1.5.npy")
    planner =  global_planner("/home/daniel/Documents/master/maps/quarray_map.ply", "/home/daniel/Documents/master/quarrayvalid_points_0.9_2.npy")
    # planner =  global_planner("/home/daniel/Documents/master/maps/quarray_map.ply", "/home/daniel/Documents/master/valid_points_0.8_2.npy")
    # planner = global_planner("/home/danitech/Documents/maps/easter_island_boy.ply", "/home/danitech/Documents/maps/island_boy_valid_points_0.85_1.5.npy")
    #planner = global_planner("/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/isaac_map.ply", "/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/valid_points_0.8_2.npy")
    #global_planner("/home/daniel/Documents/master/isaac_map.ply", False)
    rclpy.init()
    ros_planner(planner.get_global_waypoints())

    try:

        rclpy.spin(ros_planner)
        time.sleep(1)

    except KeyboardInterrupt:
        pass    
    ros_planner.destroy_node()


if __name__ == "__main__":
    main()
