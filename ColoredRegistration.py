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

class NetworkCommand(Enum):
    Nothing = 0
    GlobalRegistration = 1
    RefineRegistration = 2

class NetworkDataType(Enum):
    Response = 0
    String = 1
    PointCloud = 2
    Matrix = 3
    LoadRoom = 4
    
class NetworkResponseType(Enum):
    AllGood=0
    DataCorrupt=1

class ModelLoader:
    def reset(self):
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        self.pointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))

    def __init__(self):
        self.reset()        
        
    def append_pointcloud(self, file_name):
        loaded_pointcloud = o3d.io.read_point_cloud(file_name)
        points = np.asarray(self.pointcloud.points)
        normals = np.asarray(self.pointcloud.normals)
        points = np.append(points, np.asarray(loaded_pointcloud.points), axis = 0)
        normals = np.append(normals, np.asarray(loaded_pointcloud.normals), axis = 0)
        self.pointcloud.points = o3d.utility.Vector3dVector(points)
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)        
        
    def horisontal_crop(self, height):
        newPointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))
        new_points = []
        new_normals = []
        
        old_points = np.asarray(self.pointcloud.points)
        old_normals = np.asarray(self.pointcloud.normals)
        
        for i in range(old_points.shape[0]):
            if old_points[i,2]<height:
                new_points.append(old_points[i,:])
                new_normals.append(old_normals[i,:])
                
        newPointcloud.points = o3d.utility.Vector3dVector(np.array(new_points))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.array(new_normals))
        
        self.pointcloud = newPointcloud        
        o3d.visualization.draw_geometries([self.pointcloud], top = 30, left = 0, point_show_normal=False)
        
    def load_model(self, _range):
        for i in _range:
            self.append_pointcloud(f"EmulatedPointcloud {i}.pcd")

class RegistrationManager():
    def __init__(self):
        self.model_loader = ModelLoader()
        self.model_loader.load_model(range(1,6))
        self.source_initial_transform = np.identity(4)
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
        
    def execute_registration(self, target):
        
        full_scan = o3d.io.read_point_cloud("ReceivedPointcloud.pcd")
        
        self.target = copy.deepcopy(target)
        self.source = copy.deepcopy(self.model_loader.pointcloud)
        
        self.target.paint_uniform_color([0,1,0])
        self.source.paint_uniform_color([0,0,1])
        
        self.source.transform(np.array([[1,0,0,10],[0,1,0,0],[0,0,1,-.6],[0,0,0,1],]))
        #self.draw_registration_result(self.source, self.target, np.identity(4))

        self.source.transform(self.source_initial_transform)

        voxel_size = 1
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

        ransac_result = o3d.registration.evaluate_registration(self.source, full_scan, voxel_size*1, result_ransac)
        
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        self.result = result_icp.dot(result_ransac)
        
        final_result = o3d.registration.evaluate_registration(self.source, full_scan, voxel_size*1, self.result)
        
        print(f"::RANSAC RMSE: {ransac_result.inlier_rmse}")
        print(f"::Refinement RMSE: {final_result.inlier_rmse}")
        
        self.draw_registration_result(self.source, self.target, self.result)
        self.source = copy.deepcopy(self.model_loader.pointcloud)
        
        
reg_man = RegistrationManager()
reg_man.execute_registration(o3d.io.read_point_cloud("ReceivedPointcloud.pcd"))

