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
        
    def load_model(self, _range):
        for i in _range:
            self.pointcloud = self.append_room(self.pointcloud, i)

class RegistrationManager():
    def __init__(self):
        self.model_loader = ModelLoader()
        
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target], top = 30, left = 0, point_show_normal=False)

    def preprocess_point_cloud(self, pcd, voxel_size):    
        pcd_down = pcd.voxel_down_sample(voxel_size)
        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def prepare_dataset(self, voxel_size, source, target):    
        source_down, source_fpfh = self.preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target, voxel_size)
        return source_down, target_down, source_fpfh, target_fpfh

    def execute_global_registration(self, source, target, source_fpfh,
                                    target_fpfh, voxel_size):
        distance_threshold = voxel_size * 1.5
        result = o3d.registration.registration_ransac_based_on_feature_matching(
            source, target, source_fpfh, target_fpfh, distance_threshold,
            o3d.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.registration.RANSACConvergenceCriteria(4000000, 500)).transformation
        print(":: Ransac result: ")
        self.draw_registration_result(source, target, result)
        print(result)        
        return result

    def refine_registration(self, source, target, voxel_size, init_transform):        
        distance_threshold = voxel_size * 0.5
        source = copy.deepcopy(source)
        source.transform(init_transform)
        result = o3d.registration.registration_icp(
                source, target, distance_threshold, np.identity(4),
                o3d.registration.TransformationEstimationPointToPlane()).transformation
        print(":: ICP result: ")
        print(result)
        self.draw_registration_result(source, target, result)
        return result
        
    def execute_registration(self, target):
        self.model_loader.load_model(range(0,5)) #appends room geometries as pointclouds to the whole pointcloud        
        source = copy.deepcopy(self.model_loader.pointcloud)
        #source.paint_uniform_color([1,0,0])
        #target.paint_uniform_color([0,1,0])
        """
        source.transform(np.array([[1, 0, 0, 0],
                                        [0, 0,-1, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]]))"""
        
        self.draw_registration_result(source, target, np.identity(4))
        
        self.source = source
        self.target = target
        
        voxel_size = 1        
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)   
        
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)        
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        
        self.result = result_icp.dot(result_ransac)
        
        source = copy.deepcopy(self.model_loader.pointcloud)
        source.paint_uniform_color([1,0,0])
        self.draw_registration_result(source, target, self.result)

source = o3d.io.read_point_cloud("ReceivedPointcloud.pcd")
reg_man = RegistrationManager()
reg_man.execute_registration(source)


