import socket
import threading
import time
import struct
import open3d as o3d
import numpy as np
import copy
from enum import Enum

class NetworkDataType(Enum):
    errorType = 0
    String = 1
    PointCloud = 2
    Matrix = 3

class NetworkErrorType(Enum):
    NoError = 0
    DataCorrupt = 1    

class NetworkDataHandler:
    def __init__(self, server):
        self.server = server
    def handle_string(buffer):
        input_string = None
        try:
            input_string = buffer.decode('utf-8')
        except:
            server.send_error(NetworkErrorType.DataCorrupt)
            
    def handle_point_cloud(buffer):
        points = ''
        errorLog = ''
        chunk_size = struct.unpack_from('>i', buffer, 0)
        chunks = struck.unpack_from('>i', buffer, 4)
        offset = 8
        for chunk in range(chunks):
            for i in range(chunk_size):
                try:
                    offset += (chunk*chunk_size + i)*8*6
                    x = struct.unpack_from('>d', buffer, offset)
                    y = struct.unpack_from('>d', buffer, offset+8)
                    z = struct.unpack_from('>d', buffer, offset+16)
                    norm_x = struct.unpack_from('>d', buffer, offset+24)
                    norm_y = struct.unpack_from('>d', buffer, offset+32)
                    norm_z = struct.unpack_from('>d', buffer, offset+40)
                except:
                    errorLog+=f"\nChunk #{chunk}"                    
            print("Chunk# ", chunk, " is received")
        if len(errorLog)>0:
            print(errorLog)
            server.send_error(NetworkErrorType.DataCorrupt)
        
    def handle_network_data(self, data_type, buffer):
        if data_type == NetworkDataType.String:
            handle_string(buffer)
        elif data_type == NetworkDataType.PointCloud:
            handle_point_cloud(buffer)



class Server:
    def __init__(self, TCP_IP, TCP_PORT, FORMAT):
        self.TCP_IP = TCP_IP
        self.TCP_PORT = TCP_PORT
        self.FORMAT = FORMAT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.network_data_handler = NetworkDataHandler(self)
        self.listening_threads = []
        self.clients = []
        self.input_buffer = None
        self.output_buffer = None
        self.output_data_type = None
        
    def receive_data(conn, addr):
        connectet = True
        while connected:
            buffer_length = struct.unpack('>q', conn.recv(8))[0]
            data_type = stuct.unpack('>h', conn.recv(2))[0]
            self.input_buffer = conn.recv(buffer_length)
            self.network_data_handler.handle_network_data(data_type, self.input_buffer)
            
    def send_data():
        for conn in self.clients:
            conn = conn[0]
            conn.send(struct.pack('>q',len(self.output_buffer)))
            conn.send(struct.pack('>h',self.output_data_type.value))
            conn.send(output_buffer)
        
    def start_listening(self, max_connections):
        print("[STARTING] server  is starting...")
        self.s.listen()
        print("[Listening] Server is listening on {SERVER}")
        self.connections = 0;
        while self.connections <= max_connections:
            print(f"[ACTIVE CONNECTIONS] {threading.activeCount()-1}")
            conn, addr = self.s.accept()
            self.clients.append([conn, addr])
            thread = threading.Thread(target = receive_data, args = (conn, addr))
            self.listening_threads.append(thread)
            thread.start()
            
    def send_error(error):
        self.output_buffer = struct.pack('>h', error.value)
        self.output_data_type = NetworkDataType.errorType
        send_data()
        
    def send_string(strg):
        self.output_buffer = strg.encode('utf-8')
        self.output_data_type = NetworkDataType.String
        send_data()
        
    






str1 = struct.pack('>h',1)
str2 = struct.pack('>h',2)
str3 = str1+str2
print(str1)
print(str2)
print(type(str1))
print(struct.unpack_from('>h', str3, 2))
input()













            
'''
HEADER = 8
TCP_IP = '192.168.0.102'
TCP_PORT = 10000
BUFFER_SIZE = 1024
FORMAT = 'utf-8'
DISCONNECT_MSG = "!DISCONNECT!"

client = Server(TCP_IP, TCP_PORT, FORMAT)
client.start_listening(1)
'''

















