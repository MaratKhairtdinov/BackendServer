import open3d as o3d
import numpy as np
import copy

class ModelLader:
    def __init__(self):
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        self.pointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))
        
    def load_spaces(self, number):
        points = np.empty([0,3])
        normals = np.empty([0,3])
        for i in range(1, number+1):
            pointcloud = o3d.io.read_point_cloud(f'EmulatedPointcloud {i}.pcd')
            points = np.append(points, np.asarray(pointcloud.points), axis = 0)
            normals = np.append(normals, np.asarray(pointcloud.normals), axis = 0)
        points = np.array(points)
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(points))
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)


modelLoader = ModelLoader()
modelLoader.load_spaces(5)


o3d.visualization.draw_geometries([modelLoader.pointcloud], top = 30, left = 0, point_show_normal=True)