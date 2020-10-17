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
import matplotlib.pyplot as plt

folder_path = r"D:\Marat\Thesis\Thesis\BackendServer\RegistrationResults\HistogramsFiltered\\"

def save_histogram(column, file_name):
    file_path = folder_path+file_name
    plt.hist(column, density = True, bins = 50)
    plt.savefig(file_path)
    plt.close()

filepath = 'Plain fit RMSE.xlsx'
data = pd.read_excel(filepath).to_numpy()
#print(np.shape(data))

for i in np.arange(0, np.shape(data)[1], 4):
    number = int(i/4)
    
    distance_measure = "point to plane"
    
    save_histogram(data[:,i], f"{number} Rooms RANSAC RMSE {distance_measure}.png")
    save_histogram(data[:,i+1], f"{number} Rooms RANSAC Time {distance_measure}.png")
    save_histogram(data[:,i+2], f"{number} Rooms ICP RMSE {distance_measure}.png")
    save_histogram(data[:,i+3], f"{number} Rooms ICP Time. {distance_measure}png")