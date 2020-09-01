import open3d as o3d
import numpy as np
import copy

point_cloud = o3d.io.read_point_cloud('EmulatedPointcloud.pcd')
#point_cloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.5, max_nn = 30))
o3d.visualization.draw_geometries([point_cloud], top = 30, left = 0, point_show_normal=True)

