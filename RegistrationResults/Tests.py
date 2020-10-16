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
        #self.model_loader.load_model(range(0,5)) #appends room geometries as pointclouds to the whole pointcloud
        #self.source = copy.deepcopy(self.model_loader.pointcloud)
        
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
                                    
        #self.draw_registration_result(source, target, np.identity(4))
        distance_threshold = voxel_size * 1.5
        result = o3d.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 500)).transformation
        #self.draw_registration_result(source, target, result)
        print(":: Ransac result: ")
        print(result)
        return result

    def refine_registration(self, source, target, voxel_size, init_transform):
        distance_threshold = voxel_size * 0.5
        source = copy.deepcopy(source)
        source.transform(init_transform)
        result = o3d.registration.registration_icp(
                source, target, distance_threshold, np.identity(4),
                o3d.registration.TransformationEstimationPointToPlane()).transformation
        print(":: ICP result:")
        print(result)
        return result
        
    def execute_registration(self, source, target):
        
        #self.source_initial_transform = source_initial_transform
        
        self.target = copy.deepcopy(target)
        self.source = copy.deepcopy(source)       

        #self.source.transform(self.source_initial_transform)

        voxel_size = 1
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

        #ransac_result = o3d.registration.evaluate_registration(self.source, full_scan, voxel_size*1, result_ransac)
        
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        self.result = result_icp.dot(result_ransac)
        
        #final_result = o3d.registration.evaluate_registration(self.source, self.target, voxel_size*1, self.result) 
        
        return self.result


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

#    o3d.visualization.draw_geometries([normals_sphere],  top = 30, left = 0, point_show_normal=False)
#    o3d.visualization.draw_geometries([filtered_sphere], top = 30, left = 0, point_show_normal=False)

    filtered_pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(filtered_points))
    filtered_pointcloud.normals = o3d.utility.Vector3dVector(filtered_normals)

    o3d.io.write_point_cloud(path, filtered_pointcloud, False, False, True)
#    o3d.visualization.draw_geometries([filtered_pointcloud], top = 30, left = 0, point_show_normal=show_normals)


def draw_registration_result(source, target, transformation, image_number):    
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)        
    #o3d.visualization.draw_geometries([source_temp, target_temp], top = 30, left = 0, point_show_normal=False)
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

table = [list(range(1,11))]
counter = 0
for folder_number in range(1,5):
    column = []
    folder_path = f"C:/Users/Marat/Documents/Thesis/BackendServer/RegistrationResults/{folder_number}Rooms/"
    for number in range(1,100):
        counter+=1       

        source = o3d.io.read_point_cloud(folder_path+f"Source.pcd")
        target = o3d.io.read_point_cloud(folder_path+f"Target.pcd")
        
        init_transform   = np.load(folder_path+f"InitialTransform.npy")
        
        source.transform(init_transform)
        
        result_transform = reg_man.execute_registration(source, target)
        
        np.save(folder_path+f"ResultTransform{number}.npy", result_transform)
        
        #draw_registration_result(source, target, init_transform, counter)
        
        #target = o3d.io.read_point_cloud(folder_path+f"Target{number}.pcd")
        
        #source.transform(result_transform)
        
        #draw_registration_result(source, target, init_transform, counter)
        #draw_registration_result(source, target, result_transform, counter)
        
        reg_result = o3d.registration.evaluate_registration(source, target, 20, np.identity(4))
        print(f"::Registration RMSE: {reg_result.inlier_rmse}")
        
        column.append(reg_result.inlier_rmse)
    table.append(column)
table = np.asarray(table)
print((table))
table = np.transpose(table)


df = pd.DataFrame(table, columns = ["Test#" , "One Room", "Two Rooms", "Three Rooms", "Four Rooms"])

filepath = 'Filtered fit RMSE.xlsx'
df.to_excel(filepath, index=False)

        
        
        