import open3d as o3d
import numpy as np
import copy

class ModelLoader:    
    def reset(self):
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        self.pointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))

    def __init__(self):
        self.reset()
        
    def append_room(self, pointcloud, room_number):
        loaded_pointcloud = o3d.io.read_point_cloud(f'EmulatedPointcloud {room_number}.pcd')
        points = np.asarray(pointcloud.points)
        normals = np.asarray(pointcloud.normals)
        points = np.append(points, np.asarray(loaded_pointcloud.points), axis = 0)
        normals = np.append(normals, np.asarray(loaded_pointcloud.normals), axis = 0)
        pointcloud.points = o3d.utility.Vector3dVector(points)
        pointcloud.normals = o3d.utility.Vector3dVector(normals)
        return pointcloud
        
    def load_spaces(self, _range):
        for i in _range:
            self.pointcloud = self.append_room(self.pointcloud, i)

modelLoader = ModelLoader()
modelLoader.load_spaces(range(0,5))


o3d.visualization.draw_geometries([modelLoader.pointcloud], top = 30, left = 0, point_show_normal=True)