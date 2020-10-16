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
    source_temp = source
    target_temp = target
    source_temp.transform(transformation)        
    o3d.visualization.draw_geometries([source_temp, target_temp], top = 30, left = 0, point_show_normal=False)



for folder_number in range(1,5):
    for number in range(1,10):   

        folder_path = f"C:/Users/Marat/Documents/Thesis/BackendServer/RegistrationResults/{folder_number}Rooms/"

        source = o3d.io.read_point_cloud(folder_path+f"Source{number}.pcd")
        target = o3d.io.read_point_cloud(folder_path+f"Target{number}.pcd")  

        init_transform   = np.load(folder_path+f"InitialTransform{number}.npy")
        result_transform = np.load(folder_path+f"ResultTransform{number}.npy")

        print(init_transform)
        
        print(result_transform)

        draw_registration_result(source, target, init_transform)
        draw_registration_result(source, target, result_transform)