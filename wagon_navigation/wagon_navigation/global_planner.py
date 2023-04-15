import open3d as o3d
import numpy as np
import sys
import heapq


class global_planner():
    def __init__(self, file_path):
        self.load_ply_file(file_path)
        self.convert_to_tensor()
        self.path = np.array([(697.0, 637), (697.0, 639), (697.0, 641), (697.0, 643), (698.0, 692), (698.0, 680), (697.0, 649), (697.0, 689), (697.0, 685), (698.0, 702), (703.0, 617), (699.0, 691), (699.0, 679), (697.0, 659), (698.0, 686), (725.0, 605), (698.0, 690), (728.0, 602), (699.0, 681), (704.0, 614), (700.0, 682), (726.0, 606), (703.0, 621), (703.0, 627), (699.0, 705), (703.0, 633), (703.0, 611), (697.0, 701), (697.0, 669), (703.0, 709), (702.0, 626), (764.0, 586), (765.0, 587), (703.0, 613), (703.0, 631), (732.0, 590), (1180.0, 1190), (738.0, 588), (731.0, 591), (1146.0, 1048), (704.0, 628), (747.0, 753), (722.0, 728), (1175.0, 1171), (769.0, 575), (704.0, 622), (704.0, 710), (725.0, 731), (780.0, 556), (702.0, 610), (1175.0, 1185), (1145.0, 1049), (1102.0, 1052), (723.0, 673), (724.0, 674), (698.0, 676), (1179.0, 1177), (1171.0, 1175), (700.0, 706), (721.0, 675), (705.0, 711), (1178.0, 1176), (703.0, 625), (1187.0, 1173), (1148.0, 1046), (773.0, 561), (1181.0, 1187), (768.0, 574), (727.0, 597), (1218.0, 1154), (727.0, 601), (1173.0, 1179), (737.0, 589), (1223.0, 1161), (1201.0, 1167), (1225.0, 1159), (1190.0, 1198), (1176.0, 1186), (1205.0, 1165), (1173.0, 1183), (1153.0, 1039), (1111.0, 1051), (1190.0, 1200), (1232.0, 1158), (1291.0, 1139), (1179.0, 1189), (1294.0, 1138), (1192.0, 1030), (1190.0, 1196), (1182.0, 1192), (1298.0, 1136), (726.0, 732), (769.0, 571), (1208.0, 1164), (1193.0, 1203), (1153.0, 1041), (1197.0, 1169), (1154.0, 1040), (1184.0, 1194), (704.0, 618), (768.0, 580), (1314.0, 1124), (1480.0, 1440), (1175.0, 1179), (1149.0, 1045), (1104.0, 1052), (1184.0, 1174), (1240.0, 1156), (770.0, 566), (769.0, 565), (1481.0, 1437), (1148.0, 1044), (1263.0, 1147), (1353.0, 1101), (1360.0, 1096), (1191.0, 1199), (1484.0, 1448), (701.0, 693), (1192.0, 1202), (1172.0, 1182), (1261.0, 1149), (1147.0, 1047), (1174.0, 1180), (1254.0, 1152), (1194.0, 1170), (704.0, 632), (1171.0, 1181), (1354.0, 1100), (1354.0, 1094), (1264.0, 1148), (1258.0, 1150), (1348.0, 1108), (1480.0, 1422), (1276.0, 1144), (1303.0, 1133), (1192.0, 1200), (1209.0, 1163), (1174.0, 1184), (1317.0, 1123), (1273.0, 1145), (1249.0, 1153), (1265.0, 1141), (1317.0, 1113), (1204.0, 1160), (1188.0, 1196), (1362.0, 1092), (1182.0, 1188), (1356.0, 1098), (1278.0, 1142), (1334.0, 1104), (1279.0, 1143), (1271.0, 1129), (1351.0, 1103), (1214.0, 1162), (1350.0, 1102), (1180.0, 1178), (1284.0, 1140), (1315.0, 1125), (1350.0, 1106), (1359.0, 1097), (1310.0, 1126), (1189.0, 1033), (1185.0, 1035), (1191.0, 1201), (1336.0, 1112), (1194.0, 1028), (1347.0, 1109), (1333.0, 1105), (1321.0, 1121), (1295.0, 1137), (1331.0, 1117), (1189.0, 1197), (1188.0, 1034), (1334.0, 1114), (1333.0, 1115), (1309.0, 1127), (1332.0, 1116), (1243.0, 1155), (1306.0, 1130), (1198.0, 1168), (1360.0, 1090), (1301.0, 1135), (1305.0, 1131), (1308.0, 1128), (1202.0, 1166), (1302.0, 1134), (1327.0, 1119), (1345.0, 1111), (1346.0, 1110), (1322.0, 1122), (1237.0, 1157), (1326.0, 1120), (1195.0, 1029), (1257.0, 1151), (1349.0, 1107), (1268.0, 1146), (1304.0, 1132), (1359.0, 1095), (1361.0, 1093), (2118.0, 2080), (1203.0, 1165), (2111.0, 2083), (1479.0, 1451), (1480.0, 1452), (1479.0, 1425), (1481.0, 1441), (1481.0, 1439), (1480.0, 1450), (2112.0, 2084), (1479.0, 1429), (2124.0, 2074), (1480.0, 1430), (1206.0, 1164), (1480.0, 1438), (1480.0, 1442), (1481.0, 1443), (1482.0, 1444), (2114.0, 2082), (1328.0, 1118), (1192.0, 1172), (1511.0, 1461), (2128.0, 2072), (1481.0, 1453), (1482.0, 1454), (1481.0, 1485), (1482.0, 1486), (1483.0, 1487), (1478.0, 1420), (1479.0, 1421), (1478.0, 1422), (2125.0, 2079), (2119.0, 2081), (1483.0, 1445), (2117.0, 2081), (2113.0, 2085)])
        self.path = self.path[:,1].astype(int)
        
        #self.adjacency_list(self.mesh_map)
        
        self.get_normals(self.mesh_map)


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
        normals = np.asarray(mesh.triangle_normals)
        #normals = np.asarray(mesh.vertex_normals)
        #print("normals: \n", normals)
        # keep only the normals that are pointing up and set the other normals to 0
        #normals[normals[:, 2] < 0.5] = 0

        # create a o3d geometry lineset from self.path

        verticies = np.asarray(mesh.vertices)
        points = verticies[self.path[1:-1]]
        points_start_end = verticies[self.path[[0, -1]]]

        #print("points: \n", points)

        # put the points in a o3d pcd
        self.pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        self.pcd_start_end = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points_start_end))
        # color the points red
        self.pcd.paint_uniform_color([1, 0, 0])
        self.pcd_start_end.paint_uniform_color([0, 1, 0])

        # lines = [[i, i+1] for i in range(len(self.path)-1)]
        # colors = [[1, 0, 0] for i in range(len(self.path)-1)]
        # line_set = o3d.geometry.LineSet(
        #     points=o3d.utility.Vector3dVector(self.path),
        #     lines=o3d.utility.Vector2iVector(lines),
        # )

        # color triangle 500 to red using the o3d.visualization.MeshColorOption class
        #mesh.vertex_colors = o3d.utility.Vector3dVector(np.tile([1, 0, 0], (len(mesh.triangles), 1)))

        #mesh.triangle_normals = o3d.utility.Vector3dVector(abs(normals) < 0.5)
        #o3d.visualization.draw_geometries([mesh, pcd, pcd_start_end], point_show_normal=True)
        self.color_path(mesh, self.path)


    def adjacency_list(self, mesh):

        mesh.compute_adjacency_list()

        #print the memory size of mesh
        print("mesh size: ", sys.getsizeof(mesh.adjacency_list))

        print("adjacency list: \n", mesh.adjacency_list[0])  
        print("vertex normal 0", mesh.vertex_normals[0])


        path = self.dijkstra(mesh.adjacency_list, 32500, 32000)
        path = np.asarray(path)
        self.path = path[:,1].astype(int)




    def color_path(self, triangle_mesh, path):
        # Get the vertex and face arrays from the triangle mesh
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
        
        # print(points)
        # print(lines)

        # points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],
        #         [0, 1, 1], [1, 1, 1]]
        # lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
        #         [0, 4], [1, 5], [2, 6], [3, 7]]

        colors = [[1, 0, 0] for i in range(len(lines))]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(vertices)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(colors)

        print(np.asarray(line_set.lines))
        
        print(line_set)
        # Visualize the mesh and path together
        o3d.visualization.draw_geometries([triangle_mesh, line_set, self.pcd])
        #o3d.visualization.draw_geometries([line_set])

    def check_neighbor_feasibility(self, mesh, vertex_index, neighbor_index):
        # check if the neighbor is a wall
        normals_diff = abs(mesh.vertex_normals[vertex_index][2] - mesh.vertex_normals[neighbor_index][2])

        if mesh.vertex_normals[neighbor_index][2] < 0.5 and normals_diff < 0.2 :
            return True
        else:
            return False
            
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
