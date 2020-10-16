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

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)        
    o3d.visualization.draw_geometries([source_temp, target_temp], top = 30, left = 0, point_show_normal=False)


for number in range(1,10):
    

    source = o3d.io.read_point_cloud(f"Source{number}.pcd")
    target = o3d.io.read_point_cloud(f"Target{number}.pcd")
    
    

    init_transform = np.load(f"InitialTransform{number}.npy")
    result_transform = np.load(f"ResultTransform{number}.npy")

    print(init_transform)
    
    print(result_transform)

    draw_registration_result(source, target, init_transform)
    draw_registration_result(source, target, result_transform)