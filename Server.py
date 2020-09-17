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
        
    def append_room(self, room_number):
        loaded_pointcloud = o3d.io.read_point_cloud(f'EmulatedPointcloud {room_number}.pcd')
        points = np.asarray(self.pointcloud.points)
        normals = np.asarray(self.pointcloud.normals)
        points = np.append(points, np.asarray(loaded_pointcloud.points), axis = 0)
        normals = np.append(normals, np.asarray(loaded_pointcloud.normals), axis = 0)
        self.pointcloud.points = o3d.utility.Vector3dVector(points)
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)
        o3d.visualization.draw_geometries([self.pointcloud], top = 30, left = 0, point_show_normal=True)
        
    def load_model(self, _range):
        for i in _range:
            self.append_room(i)
        

class RegistrationManager():
    def __init__(self):
        self.model_loader = ModelLoader()
        self.source_initial_transform = np.identity(4)
        #self.model_loader.load_model(range(0,5)) #appends room geometries as pointclouds to the whole pointcloud
        #self.source = copy.deepcopy(self.model_loader.pointcloud)
        
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
        print(":: ICP result:")
        print(result)
        return result
        
    def execute_registration(self, command, target):
        print(f"Command: {command}")
        self.target = target
        self.source = copy.deepcopy(self.model_loader.pointcloud)
        self.source.transform(self.source_initial_transform)
        #self.draw_registration_result(self.source, self.target, np.identity(4))
        voxel_size = 1
        source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset(voxel_size, self.source, self.target)
        result_ransac = np.identity(4)        
        if command == NetworkCommand.GlobalRegistration.value:
            print("RANSAC")
            result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        #self.draw_registration_result(self.source, self.target, result_ransac)
        result_icp = self.refine_registration(self.source, self.target, voxel_size, result_ransac)
        
        self.result = result_icp.dot(result_ransac)
        #print(self.result)
        
        self.source.paint_uniform_color([1,0,0])        
        #self.draw_registration_result(self.source, self.target, self.result)
        #self.source = copy.deepcopy(self.model_loader.pointcloud)
        #self.draw_registration_result(self.source, self.target, np.identity(4))

class NetworkDataHandler:
    def __init__(self, client):
        self.client = client
        self.last_buffer_sent = None;
        self.last_data_type_sent = None;
        self.pointcloud = None
        self.registration_manager = RegistrationManager()

    def handle_response(self, buffer):
        response = struct.unpack('>h', buffer)
        if response == NetworkResponseType.AllGood.value:
            pass
        elif response == NetworkResponseType.DataCorrupt.value:
            self.client.send(self.last_data_type_sent, self.last_buffer_sent)
            
    def handle_matrix(self, buffer):
        matrix = np.identity(4)
        offset = 0
        response = None
        try:
            for column in range(4):
                for row in range(4):                                    
                    number = struct.unpack_from('f', buffer, offset)[0]                    
                    matrix[column, row] = number            
                    offset+=4
            response = NetworkResponseType.AllGood
            print(matrix)
        except:
            response = NetworkResponseType.DataCorrupt
        self.client.send_response(response)
        self.registration_manager.source_initial_transform = matrix
        
        
    def handle_string(self, buffer):
        response = None
        try:
            print(buffer.decode('utf-8'))
            response = NetworkResponseType.AllGood
        except:
            response = NetworkResponseType.DataCorrupt            
        self.client.send_response(response)

    def write_PCD(self, pointcloud):
        o3d.io.write_point_cloud("ReceivedPointcloud.pcd", pointcloud, False, False, True)
        print("[PCD_WRITTEN_INTO_FILE]")

    def handle_point_cloud(self, command, buffer):
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
            points   = np.array(points)
            normals  = np.array(normals)
        except Exception:
            response = NetworkResponseType.DataCorrupt
        self.client.send_response(response)
        self.pointcloud = o3d.geometry.PointCloud(points = o3d.utility.Vector3dVector(points))
        self.pointcloud.normals = o3d.utility.Vector3dVector(normals)

        self.write_PCD(self.pointcloud)
        self.registration_manager.execute_registration(command, self.pointcloud)
        self.client.send_matrix(self.registration_manager.result)
        
    def handle_room_request(self, buffer):
        print("RoomRequested")
        self.registration_manager.model_loader.append_room(struct.unpack('i', buffer)[0])
        
        
    def handle_network_data(self, command, data_type, buffer):
    
        if data_type == NetworkDataType.String.value:
            self.handle_string(buffer)
        elif data_type == NetworkDataType.Response.value:
            self.handle_response(buffer)
        elif data_type == NetworkDataType.PointCloud.value:            
            self.handle_point_cloud(command, buffer)                
        elif data_type == NetworkDataType.Matrix.value:
            self.handle_matrix(buffer)
        elif data_type == NetworkDataType.LoadRoom.value:
            self.handle_room_request(buffer)
        

class InputGateThread(threading.Thread):
    def __init__(self, client, ip, port):
        self.client = client
        self.IP = ip
        self.PORT = port        
        self.gate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.receiving = True
        threading.Thread.__init__(self)
        
        print("[ INPUT GATE INITIALIZED]")
        
    def run(self):                
        self.gate.bind((self.IP, self.PORT))
        self.gate.listen()
        print("[ Input Gate is listening...]")
        self.conn, self.address = self.gate.accept()
        print("[ CLIENT CONNECTED TO INPUT GATE]")
        self.receive()
        
    def receive(self):        
        while self.receiving:
            input_buff = []
            command = struct.unpack('>h', self.conn.recv(2))[0]
            print(command)
            data_type  = struct.unpack('>h', self.conn.recv(2))[0]
            chunk_size = struct.unpack('>i', self.conn.recv(4))[0]
            chunks     = struct.unpack('>i', self.conn.recv(4))[0]
            residual   = struct.unpack('>i', self.conn.recv(4))[0]
            print(f"{chunks} chunks of size {chunk_size} expected")
            errorLog = ""
            for i in range(chunks):
                received = False
                while not received:
                    chunk = self.conn.recv(chunk_size)
                    if len(chunk)==chunk_size:
                        input_buff.append(chunk)
                        self.conn.send(struct.pack('h', NetworkResponseType.AllGood.value))
                        received = True
                        print(f"Chunk #{i} received")
                    else:
                        errorLog+="\nChunk #{i} corrupt"
                        self.conn.send(struct.pack('h', NetworkResponseType.DataCorrupt.value))
            rest_data = self.conn.recv(residual)
            last_chunk_received = False
            while not last_chunk_received:
                if len(rest_data)== residual:
                    input_buff.append(rest_data)
                    self.conn.send(struct.pack('h', NetworkResponseType.AllGood.value))
                    last_chunk_received = True
                else:
                    errorLog+="\nLast chunk corrupt"
                    self.conn.send(struct.pack('h', NetworkResponseType.DataCorrupt.value))            
            input_buff = b''.join(input_buff)
            self.client.network_data_handler.handle_network_data(command, data_type, input_buff)
            
            
class OutputGateThread(threading.Thread):
    def __init__(self, client, ip, port):
        self.client = client
        self.IP = ip
        self.PORT = port
        self.gate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.receiving = True
        threading.Thread.__init__(self)
        print("[OUTPUT GATE INITIALIZED]")
        
    def run(self):
        self.gate.bind((self.IP, self.PORT))
        self.gate.listen()
        print("[Output Gate is listening...]")
        self.conn, self.address = self.gate.accept()
        print("[CLIENT CONNECTED TO OUTPUT GATE]")
        
    def send(self, data_type, buffer):
        self.conn.send(struct.pack('h',data_type.value))
        length_buffer = struct.pack('i',len(buffer))
        self.conn.send(length_buffer)
        self.conn.send(buffer)
        print(f"data sent {data_type.value}, buffer length: {len(buffer)}")
        
class Client:
    def __init__(self, HOST, INPUT_PORT, OUTPUT_PORT):
        print("[STARTING SERVER]")
        self.input_gate = InputGateThread(self, HOST, INPUT_PORT)
        self.output_gate = OutputGateThread(self, HOST ,OUTPUT_PORT)
        self.network_data_handler = NetworkDataHandler(self)
    def start(self):        
        self.input_gate.start()
        self.output_gate.start()        
    def send_matrix(self, matrix):
        buffer = []
        for column in range(matrix.shape[1]):
            for row in range(matrix.shape[0]):
                buffer.append(struct.pack('d',matrix[row, column]))
        buffer = b''.join(buffer)
        self.output_gate.send(NetworkDataType.Matrix, buffer)
    def send_response(self, response):
        self.output_gate.send(NetworkDataType.Response, struct.pack('h', response.value))
        print(f"{response.name} sent back")
        
class Server:
    def __init__(self, HOST, INPUT_PORT, OUTPUT_PORT):
        self.client = Client(HOST, INPUT_PORT, OUTPUT_PORT)
    def start_clients(self):
        self.client.start()
        
        
        
HOST = "192.168.0.100"
INPUT_PORT  = 6000
OUTPUT_PORT = 5000

server = Server(HOST, INPUT_PORT, OUTPUT_PORT)
server.start_clients()