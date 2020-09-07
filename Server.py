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

class NetworkDataType(Enum):
    Response = 0
    String = 1
    PointCloud = 2

class NetworkResponseType(Enum):
    AllGood=0
    DataCorrupt=1

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
        return result
        
    def execute_registration(self, target):
        self.model_loader.load_model(range(0,5)) #appends room geometries as pointclouds to the whole pointcloud
        source = copy.deepcopy(self.model_loader.pointcloud)
        source.paint_uniform_color([1,0,0])
        target.paint_uniform_color([0,1,0])
        
        self.source = source
        self.target = target
        
        voxel_size = 1        
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)   
        
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)        
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        
        self.result = result_icp.dot(result_ransac)
        print(self.result)
        
        source = copy.deepcopy(self.model_loader.pointcloud)
        source.paint_uniform_color([1,0,0])
        self.draw_registration_result(source, target, self.result)
        

class NetworkDataHandler:
    def __init__(self, server):
        self.server = server
        self.last_buffer_sent = None;
        self.last_data_type_sent = None;
        self.pointcloud = None
        self.registration_manager = RegistrationManager()

    def handle_response(self, buffer):
        response = struct.unpack('>h', buffer)
        if response == NetworkResponseType.AllGood.value:
            pass
        elif response == NetworkResponseType.DataCorrupt.value:
            self.server.send(self.conn, self.addr, self.last_data_type_sent, self.last_buffer_sent)
            
    def handle_string(self, buffer):
        response = None
        try:
            print(buffer.decode('utf-8'))
            response = NetworkResponseType.AllGood
        except:
            response = NetworkResponseType.DataCorrupt            
        self.server.send_response(self.conn, self.addr, response)

    def write_PCD(self, pointcloud):
        o3d.io.write_point_cloud("ReceivedPointcloud.pcd", pointcloud, False, False, True)
        print("[PCD_WRITTEN_INTO_FILE]")


    def handle_point_cloud(self, buffer):
        points_list = ""
        offset = 4
        points_number = struct.unpack_from('i', buffer, 0)[0]
        points  = []
        normals = []
        response = None
        try:
            for i in range(points_number):
                points.append([struct.unpack_from('f', buffer, offset)[0],     struct.unpack_from('f', buffer, offset+8)[0],   struct.unpack_from('f', buffer, offset+4)[0]])
                normals.append([struct.unpack_from('f', buffer, offset+12)[0], struct.unpack_from('f', buffer, offset+20)[0], struct.unpack_from('f', buffer, offset+16)[0]])
                offset += 24
            response = NetworkResponseType.AllGood
            points   =  np.array(points)
            normals  = np.array(normals)
        except Exception:
            response = NetworkResponseType.DataCorrupt
        #print(f"{np.shape(points)}, {np.shape(normals)}")
        self.server.send_response(self.conn, self.addr, response)
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(points))
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)
        
        self.write_PCD(self.pointcloud)
        self.registration_manager.execute_registration(self.pointcloud)
        
    def handle_network_data(self, conn, addr, data_type, buffer):
        self.conn = conn
        self.addr = addr
    
        if data_type == NetworkDataType.String.value:
            self.handle_string(buffer)
        elif data_type == NetworkDataType.Response.value:
            self.handle_response(buffer)
        elif data_type == NetworkDataType.PointCloud.value:
            self.handle_point_cloud(buffer)
        

class Server:
    def __init__(self, TCP_IP, PORT):
        print("[STARTING] Server is starting")
        self.TCP_IP = TCP_IP
        self.PORT   = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.TCP_IP, self.PORT))
        self.data_handler = NetworkDataHandler(self)
        self.handlers = []
        self.receiving = False
        
    def send(self, conn, addr, data_type, buffer):
        conn.send(struct.pack('>h',data_type.value))
        length_buffer = struct.pack('i',len(buffer))
        conn.send(length_buffer)
        conn.send(buffer)
        print("data sent")

    def send_matrix(self, conn, addr, matrix):
        pass
        
    def send_response(self, conn, addr, response):        
        self.send(conn, addr, NetworkDataType.Response, struct.pack('>h', response.value))
        print(f"{response.name} sent back")
        
    def listen(self, conn, addr):
        receiving = True        
        while receiving:
            input_buff = []
            data_type  = struct.unpack('>h', conn.recv(2))[0]
            chunk_size = struct.unpack('>i', conn.recv(4))[0]
            chunks     = struct.unpack('>i', conn.recv(4))[0]
            residual   = struct.unpack('>i', conn.recv(4))[0]
            print(f"{chunks} chunks of size {chunk_size} expected")
            errorLog = ""
            for i in range(chunks):
                received = False
                while not received:
                    chunk = conn.recv(chunk_size)
                    if len(chunk)==chunk_size:
                        input_buff.append(chunk)
                        conn.send(struct.pack('>h', NetworkResponseType.AllGood.value))
                        received = True
                        #print(f"Chunk #{i} received")
                    else:
                        errorLog+="\nChunk #{i} corrupt"
                        conn.send(struct.pack('>h', NetworkResponseType.DataCorrupt.value))                
            rest_data = conn.recv(residual)
            last_chunk_received = False
            while not last_chunk_received:
                if len(rest_data)== residual:
                    input_buff.append(rest_data)
                    conn.send(struct.pack('>h', NetworkResponseType.AllGood.value))
                    last_chunk_received = True
                else:
                    errorLog+="\nLast chunk corrupt"
                    conn.send(struct.pack('>h', NetworkResponseType.DataCorrupt.value))                
            
            input_buff = b''.join(input_buff)

            print(f"Data received, type: {data_type}")
            print(f"ErrorLog:{errorLog}")
            self.data_handler.handle_network_data(conn, addr, data_type, input_buff)
        
    def start_listening(self, number_of_clients):
        self.s.listen()
        print("[LISTENING] Server is listening on{SERVER}")
        for i in range(number_of_clients):
            print(f"[ACTIVE CONNECTIONS]{threading.activeCount()-1}")
            server = self
            conn, addr = self.s.accept()
            #self.handlers.append(NetworkDataHandler(server, conn, addr))
            thread = threading.Thread(target = self.listen, args = (conn, addr))
            thread.start()

TCP_IP = "192.168.0.100"
PORT = 10000

server = Server(TCP_IP, PORT)
server.start_listening(5)

input()
