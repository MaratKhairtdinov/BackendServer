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

class NetworkDataHandler:    
    def __init__(self, server):
        self.server = server
        self.last_buffer_sent = None;
        self.last_data_type_sent = None;
        
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

    def write_PCD(self, points_list, number):
        header = f""" VERSION .7
FIELDS x y z normal_x normal_y normal_z
SIZE 4 4 4 4 4 4
TYPE F F F F F F
COUNT 1 1 1 1 1 1
WIDTH {number}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {number}
DATA ascii\n"""        
        header += points_list
        name = 'ReceivedPointCloud.pcd'                                  
        file = open(name,"w")
        file.write(header)
        file.close()
        print("[PCD_WRITTEN_INTO_FILE]")
        #visualize_PCD(name)

    def handle_point_cloud(self, buffer):
        points_list = ""
        offset = 4
        points = struct.unpack_from('i', buffer, 0)[0]
        
        response = None
        try:
            for i in range(points):
                points_list += f"{struct.unpack_from('f', buffer, offset)[0]} "
                points_list += f"{struct.unpack_from('f', buffer, offset+8)[0]} "
                points_list += f"{struct.unpack_from('f', buffer, offset+4)[0]} "
                points_list += f"{struct.unpack_from('f', buffer, offset+12)[0]} "
                points_list += f"{struct.unpack_from('f', buffer, offset+20)[0]} "
                points_list += f"{struct.unpack_from('f', buffer, offset+16)[0]}\n"
                offset += 24
            response = NetworkResponseType.AllGood            
        except Exception:            
            response = NetworkResponseType.DataCorrupt            
        self.server.send_response(self.conn, self.addr, response)
        self.write_PCD(points_list, points) 
        
        
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
                        print(f"Chunk #{i} received")
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

TCP_IP = "192.168.0.102"
PORT = 10000

server = Server(TCP_IP, PORT)
server.start_listening(5)

input()
