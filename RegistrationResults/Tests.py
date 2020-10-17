import socket
import threading
import time
import struct
import open3d as o3d
import numpy as np
import copy
import sys
import traceback
from enum import Enum
import pandas as pd


img_path = f"C:/Users/Marat/Documents/Thesis/BackendServer/RegistrationResults/Screenshots/"

class RegistrationManager():
    def __init__(self):
        pass
        
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.transform(transformation)        
        o3d.visualization.draw_geometries([source_temp, target_temp], top = 30, left = 0, point_show_normal=False)

    def preprocess_point_cloud(self, pcd, voxel_size):    
        pcd_down = pcd.voxel_down_sample(voxel_size)
        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh
        
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

    def prepare_dataset(self, voxel_size, source, target):
        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        
        source_down.paint_uniform_color([0,0,1])
        target_down.paint_uniform_color([0,1,0])
        
        return source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(self, source, target, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        registration_time = time.time()
        result = o3d.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
        return result, time.time()-registration_time

    def refine_registration(self, source, target, voxel_size, init_transform):
        distance_threshold = voxel_size * 0.5
        source = copy.deepcopy(source)
        source.transform(init_transform)
        
        registration_time = time.time()
        result = o3d.registration.registration_icp(
                source, target, distance_threshold, np.identity(4),
                o3d.registration.TransformationEstimationPointToPoint())        
        return result, time.time()-registration_time
        
    def execute_registration(self, source, target):
        self.target = copy.deepcopy(target)
        self.source = copy.deepcopy(source)

        voxel_size = 1
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)

        result_ransac, ransac_time = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

        ransac_rmse = o3d.registration.evaluate_registration(self.source, self.target, voxel_size*1, result_ransac.transformation).inlier_rmse

        result_icp, icp_time = self.refine_registration(self.source, self.target, voxel_size, result_ransac.transformation)
        self.result = result_icp.transformation.dot(result_ransac.transformation)

        return self.result, ransac_rmse, result_icp.inlier_rmse, ransac_time, icp_time


def filter_pointcloud(pointcloud, path):
    show_normals = False

    points = np.asarray(pointcloud.points)
    normals= np.asarray(pointcloud.normals)

    normals_sphere = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(normals))
    normals_sphere.normals = o3d.utility.Vector3dVector(points)

    pcd = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.asarray(normals_sphere.normals)))
    pcd.normals = o3d.utility.Vector3dVector(np.asarray(normals_sphere.points))

    labels = np.array(normals_sphere.cluster_dbscan(eps=0.02, min_points=100, print_progress=True))

    max_label = labels.max()

    filtered_normals = np.asarray(normals_sphere.points)[labels>=0]
    filtered_points = np.asarray(normals_sphere.normals)[labels>=0]

    filtered_sphere = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(filtered_normals))
    filtered_sphere.normals = o3d.utility.Vector3dVector(filtered_points)

    filtered_pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(filtered_points))
    filtered_pointcloud.normals = o3d.utility.Vector3dVector(filtered_normals)

    o3d.io.write_point_cloud(path, filtered_pointcloud, False, False, True)



def draw_registration_result(source, target, transformation, image_number):    
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)        

    vis = o3d.visualization.Visualizer()
    vis.poll_events()
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)
    vis.run()
    vis.capture_screen_image(img_path+f"image_{image_number}.jpg")
    vis.close()
    vis.destroy_window()

reg_man = RegistrationManager()

number_of_tests = 1000


table = []
counter = 0

for number in range(1, number_of_tests):
    row = []
    for folder_number in range(1,5):
        counter+=1
        folder_path = f"D:/Marat/Thesis/Thesis/BackendServer/RegistrationResults/{folder_number}Rooms/"
        source = o3d.io.read_point_cloud(folder_path+f"Source.pcd")
        target = o3d.io.read_point_cloud(folder_path+f"Target.pcd")
        
        init_transform   = np.load(folder_path+f"InitialTransform.npy")
        source.transform(init_transform)
        
        result_transform, ransac_rmse, icp_rmse, ransac_time, icp_time = reg_man.execute_registration(source, target)
        
        row.append(ransac_rmse)
        row.append(ransac_time)
        row.append(icp_rmse)
        row.append(icp_time)

    table.append(row)
    print(f"Text# {number}")

table = np.asarray(table)
print(len(table))
print(len(table[0]))

print(np.shape(table))
print(table)

def save_histogram(column):
    plt.hist(column)
    plt.show()
df = pd.DataFrame(table, columns = ["One Room RANSAC", "Time", "One Room Refined", "Time", "Two Rooms RANSAC", "Time", "Two Rooms Refined", "Time", "Three Rooms RANSAC", "Time", "Three Rooms Refined", "Time", "Four Rooms RANSAC", "Time", "Four Rooms Refined", "Time"])

filepath = 'Plain fit, point-point RMSE.xlsx'
df.to_excel(filepath, index=False)

        
        
        