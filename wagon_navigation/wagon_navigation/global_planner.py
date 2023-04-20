import open3d as o3d
import numpy as np
import sys
import heapq
import time
import copy
from articulation_controller import articulationController


class global_planner():
    def __init__(self, file_path, load_valid_points_from_path=False):
        self.load_ply_file(file_path)
        self.convert_to_tensor()
        
        #self.adjacency_list(self.mesh_map)

        normal_mesh = self.get_normals(self.mesh_map)


        self.adj_list = self.adjacency_list(self.mesh_map)

        if load_valid_points_from_path:
            self.load_valid_verticies(load_valid_points_from_path)
        else:
            self.determine_valid_vertices(0.8, 2, "/home/daniel/Documents/master/")
        
        self.start_stop_chooser(self.mesh_map)

        path = self.a_star(self.adj_list, self.points[0], self.points[1]) # start at 7000
        if path is None:
            print("No path found")
            sys.exit()
        
        global_waypoints = self.convert_path(path)

        self.color_path(normal_mesh, path)

        self.articulation_controller = articulationController(global_waypoints)
        self.articulation_controller.run()


    def load_ply_file(self, file_path):
        self.mesh_map = o3d.io.read_triangle_mesh(file_path)
        # o3d.visualization.draw_geometries([self.mesh_map])

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
        self.valid_points = np.load(file_path)

    def determine_valid_vertices(self, normal_z_threshold, invalidation_radius=1, path="/home/daniel/Documents/master/"):
        self.valid_points = np.ones(self.verticies.shape[0])
        self.visited_points = np.ones(self.verticies.shape[0]) * -1

        self.valid_points[self.vertex_normals[:, 2] < normal_z_threshold] = 0
        print("valid points: ", np.count_nonzero(self.valid_points))
        invalid_points = np.where(self.valid_points == 0)[0] # create list of "hard" invalid point indexes

        sys.setrecursionlimit(2000)

        # Can possibly be optimized by only looking at invalid points that have valid points as neighbors
        for point_idx in invalid_points:
            print("invalid point: ", point_idx)
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
        self.points = np.array([], dtype=int)
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(mesh)
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
        
        if len(self.points) == 2:
            self.vis.destroy_window()


    

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

    def heuristic_cost_estimate(self, start_idx, end_idx):

        start = self.verticies[start_idx]
        end = self.verticies[end_idx]
        return np.linalg.norm(start-end)
    

    
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

                cost_to_neighbor = g_scores[current_node[1]] + self.heuristic_cost_estimate(current_node[1], neighbor_index)
                if neighbor_index not in g_scores or cost_to_neighbor < g_scores[neighbor_index]:
                    g_scores[neighbor_index] = cost_to_neighbor
                    heuristic_to_goal = self.heuristic_cost_estimate(neighbor_index, goal_index)
                    f_score = cost_to_neighbor + heuristic_to_goal
                    neighbor_node = (f_score, neighbor_index, current_node)
                    heapq.heappush(frontier, neighbor_node)
        
        return None
    
    def convert_path(self, path):

        global_path = self.verticies[path]
        print(global_path)


        return global_path




def main():
#    global_planner("/home/daniel/Documents/master/isaac_map.ply", "/home/daniel/Documents/master/valid_points.npy")
    global_planner("/home/daniel/Documents/master/isaac_map.ply", "/home/daniel/Documents/master/valid_points_0.8_2.npy")
    #global_planner("/home/daniel/Documents/master/isaac_map.ply", False)



if __name__ == "__main__":
    main()
