import open3d as o3d
import numpy as np
import sys
import heapq
import time
import copy


class global_planner():
    def __init__(self, file_path):
        self.load_ply_file(file_path)
        self.convert_to_tensor()
        
        #self.adjacency_list(self.mesh_map)

        normal_mesh = self.get_normals(self.mesh_map)

        self.start_stop_chooser(self.mesh_map)

        self.adj_list = self.adjacency_list(self.mesh_map)

        self.determine_valid_vertices(0.8)

        path = self.a_star(self.adj_list, self.points[0], self.points[1]) # start at 7000
        if path is None:
            print("No path found")
            sys.exit()

        self.color_path(normal_mesh, path)


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

        print("adjacency list: \n", adj_list) 
        print("adjacency list index: \n", adj_list[1][1])


        return adj_list
        #path = self.dijkstra(mesh.adjacency_list, 32500, 32000)
        
        # path = np.asarray(path)
        # self.path = path[:,1].astype(int)


    # def path_to_lineset(self, path):
    def determine_valid_vertices(self, normal_z_threshold):
        self.valid_points = np.ones(self.verticies.shape[0])
        self.visited_points = np.ones(self.verticies.shape[0]) * -1

        self.valid_points[self.vertex_normals[:, 2] < normal_z_threshold] = 0
        print("valid points: ", np.count_nonzero(self.valid_points))
        invalid_points = np.where(self.valid_points == 0)[0] # create list of "hard" invalid point indexes

        for point_idx in invalid_points:
            #print("invalid point: ", point_idx)
            self.recursive_i = 0
            self.invalidate_in_radius(self.adj_list, 1, point_idx, point_idx) # point_idx is given twice, since we have to compare the distance from the oringal invalid point
            #print("recursive calls: ", self.recursive_i)

        print("valid points after recursion: ", np.count_nonzero(self.valid_points))

    def invalidate_in_radius(self, adj_list, radius, point_idx, neighbor_idx):
        self.recursive_i += 1
        if self.visited_points[point_idx] == point_idx:
            return
        
        self.visited_points[point_idx] = point_idx

        if np.linalg.norm(self.verticies[neighbor_idx] - self.verticies[point_idx]) > radius:
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
        self.vis = o3d.visualization.VisualizerWithVertexSelection()
        self.vis.create_window()
        self.vis.add_geometry(mesh)
        self.vis.register_selection_changed_callback(self.start_stop_handler)
        self.vis.run()  # user picks points

    def start_stop_handler(self):
        print(self.vis.get_render_option())
        points= self.vis.get_picked_points()
        print("picked points: ", points[0].index)
        if len(points) == 2:
            self.vis.destroy_window()
            self.points = np.array([points[0].index, points[1].index])


    

    def check_neighbor_feasibility(self, adj_list, vertex_index, neighbor_index):
        # check if the neighbor is a wall
        normals_diff = abs(self.vertex_normals[vertex_index][2] - self.vertex_normals[neighbor_index][2])



        #if self.vertex_normals[neighbor_index][2] > 0.8 and normals_diff < 0.2:
        if self.valid_points[vertex_index] == 1:
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

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from[:,1]:
            current = came_from[current]
            total_path.append(current)
        return total_path
    
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

    def dijkstra(self, adjacency_list, start, end):
        # Initialize the distance of all nodes to infinity
        num_nodes = len(adjacency_list)
        distance = [float('inf')] * num_nodes
        distance[start] = 0
        
        # Use a min heap to keep track of nodes with smallest distance
        heap = [(0, start)]
        
        while heap:
            # Get the node with the smallest distance
            curr_dist, curr_node = heapq.heappop(heap)
            
            # If we've reached the end node, return the shortest distance
            if curr_node == end:
                return heap
            
            #print("curr_node: ", curr_node)
            #print("adjacency_list[curr_node]: ", adjacency_list[curr_node]  )
            # Otherwise, update the distances of adjacent nodes
            #print("heap: ", heap)
            for neighbor in adjacency_list[curr_node]:
                #print("neighbor: ", neighbor)
                new_dist = curr_dist + np.linalg.norm(curr_node-neighbor)
                if new_dist < distance[neighbor]:
                    distance[neighbor] = new_dist
                    heapq.heappush(heap, (new_dist, neighbor))
        
        # If we've searched the entire graph and haven't found the end node, return None
        return None

def main():
    global_planner("/home/daniel/Documents/master/isaac_map.ply")



if __name__ == "__main__":
    main()
