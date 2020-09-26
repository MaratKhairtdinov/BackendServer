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
        
    def horisontal_crop(self, pointcloud, height):
        newPointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))
        new_points = []
        new_normals = []
        
        old_points = np.asarray(pointcloud.points)
        old_normals = np.asarray(pointcloud.normals)
        
        for i in range(old_points.shape[0]):
            if old_points[i,2]<height:
                new_points.append(old_points[i,:])
                new_normals.append(old_normals[i,:])
                
        newPointcloud.points = o3d.utility.Vector3dVector(np.array(new_points))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.array(new_normals))
        
        return newPointcloud        
       
    def load_spaces(self, _range):
        for i in _range:
            self.pointcloud = self.append_room(self.pointcloud, i)

modelLoader = ModelLoader()
modelLoader.load_spaces(range(1,13))
received_pointcloud = o3d.io.read_point_cloud("ReceivedPointcloud.pcd")
combined_pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.append(np.asarray(received_pointcloud.points), np.asarray(modelLoader.pointcloud.points), axis = 0)))
combined_pointcloud.normals = o3d.utility.Vector3dVector(np.append(np.asarray(received_pointcloud.normals), np.asarray(modelLoader.pointcloud.normals), axis = 0))
o3d.visualization.draw_geometries([combined_pointcloud], top = 30, left = 0, point_show_normal=False)

o3d.visualization.draw_geometries([modelLoader.pointcloud], top = 30, left = 0, point_show_normal=False)
