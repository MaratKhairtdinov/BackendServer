import open3d as o3d
import numpy as np
import copy

class ModelLoader:    
    def reset(self):
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        self.pointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))

    def __init__(self):
        self.reset()
        
    def append_room(self, file_name):
        loaded_pointcloud = o3d.io.read_point_cloud(file_name)
        points = np.asarray(self.pointcloud.points)
        normals = np.asarray(self.pointcloud.normals)
        points = np.append(points, np.asarray(loaded_pointcloud.points), axis = 0)
        normals = np.append(normals, np.asarray(loaded_pointcloud.normals), axis = 0)
        self.pointcloud.points = o3d.utility.Vector3dVector(points)
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)
        
    def append_pointcloud(self, pointcloud):
        points = np.asarray(self.pointcloud.points)
        normals = np.asarray(self.pointcloud.normals)
        points = np.append(points, np.asarray(pointcloud.points), axis = 0)
        normals = np.append(normals, np.asarray(pointcloud.normals), axis = 0)
        self.pointcloud.points = o3d.utility.Vector3dVector(points)
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)
        
    def horisontal_crop(self, pointcloud, height):
        newPointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.empty([0,3]))
        new_points = []
        new_normals = []
        
        old_points = np.asarray(pointcloud.points)
        old_normals = np.asarray(pointcloud.normals)
        
        for i in range(old_points.shape[0]):
            if old_points[i,2]>height:
                new_points.append(old_points[i,:])
                new_normals.append(old_normals[i,:])
                
        newPointcloud.points = o3d.utility.Vector3dVector(np.array(new_points))
        newPointcloud.normals = o3d.utility.Vector3dVector(np.array(new_normals))
        
    def load_model(self, _range):
        for i in _range:
            self.append_room(f"EmulatedPointcloud {i}.pcd")
        
        
            
    
        

class RegistrationManager():
    def __init__(self):
        self.model_loader = ModelLoader()
        
    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target], top = 30, left = 0, point_show_normal=True)

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
        self.model_loader.load_model(range(1,6)) #appends room geometries as pointclouds to the whole pointcloud        
        source = copy.deepcopy(self.model_loader.pointcloud)
        #source.paint_uniform_color([1,0,0])
        #target.paint_uniform_color([0,1,0])

        self.draw_registration_result(source, target, np.identity(4))
        
        self.source = source
        self.target = target
        
        voxel_size = 1        
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)   
        
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)        
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        
        self.result = result_icp.dot(result_ransac)
        
        #registered_source = copy.deepcopy(self.target)
        #registered_source.transform(self.result)
        
        self.model_loader.append_pointcloud(registered_source)
        o3d.io.write_point_cloud("CombinedPointcloud.pcd", reg_man.model_loader.pointcloud, False, False, True)
        
        #source = copy.deepcopy(self.model_loader.pointcloud)
        #source.paint_uniform_color([1,0,0])
        #self.draw_registration_result(source, target, self.result)
        

        

source = o3d.io.read_point_cloud("FilteredPointcloud.pcd")
reg_man = RegistrationManager()
reg_man.execute_registration(source)

#o3d.io.write_point_cloud("CombinedPointcloud.pcd", reg_man.model_loader.pointcloud, False, False, True)


