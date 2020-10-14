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
        #self.pointcloud = self.horisontal_crop(self.pointcloud, 1.5)

    
            
    def segment_planes(self):
        plane_model, inliers = self.pointcloud.segment_plane(distance_threshold=0.02,
                                         ransac_n=3,
                                         num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = self.pointcloud.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = self.pointcloud.select_by_index(inliers, invert=True)
        #o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], top = 30, left = 0, point_show_normal=False)

modelLoader = ModelLoader()
modelLoader.load_spaces(range(1,13))
o3d.visualization.draw_geometries([modelLoader.pointcloud], top = 30, left = 0, point_show_normal=True)

pointcloud = o3d.io.read_point_cloud("ReceivedPointcloud.pcd")
o3d.visualization.draw_geometries([pointcloud], top = 30, left = 0, point_show_normal=True)
#modelLoader.segment_planes()