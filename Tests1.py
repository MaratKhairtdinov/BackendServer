import open3d as o3d
import numpy as np
import copy

class PlaneSegmenter:
    def __init__(self):
        self.inliers = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
        self.outliers = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(np.empty([0,3])))
    
    def extract_planes(self, pointcloud):
        plane_model, inlier_indeces = pointcloud.segment_plane(distance_threshold = 0.1, ransac_n = 3, num_iterations = 1000)
        self.inliers+=pointcloud.select_by_index(inlier_indeces)
        self.outliers=pointcloud.select_by_index(inlier_indeces, invert=True)
        
pointcloud = o3d.io.read_point_cloud("FilteredPointcloud.pcd")
plane_segmenter = PlaneSegmenter()
plane_segmenter.extract_planes(pointcloud)
plane_segmenter.extract_planes(plane_segmenter.outliers)
plane_segmenter.extract_planes(plane_segmenter.outliers)
plane_segmenter.extract_planes(plane_segmenter.outliers)
plane_segmenter.inliers.paint_uniform_color([1,0,0])
plane_segmenter.outliers.paint_uniform_color([0,1,0])
o3d.visualization.draw_geometries([plane_segmenter.inliers, plane_segmenter.outliers], top = 30, left = 0, point_show_normal=True)