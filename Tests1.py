import open3d as o3d
import numpy as np
import copy

pointcloud = o3d.io.read_point_cloud("ReceivedPointcloud.pcd")


o3d.visualization.draw_geometries([pointcloud], top = 30, left = 0, point_show_normal=True)