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

class NetworkResponseType(Enum):
    AllGood=0
    DataCorrupt=1

class NetworkDataHandler:    
    def __init__(self, server):
        self.server = server
        #self.conn = conn
        #self.addr = addr
        self.last_buffer_sent = None;
        self.last_data_type_sent = None;
        
    def handle_response(self, buffer):
        response = struct.unpack('>h', buffer)
        if response == NetworkResponseType.AllGood.value:
            pass
        elif response == NetworkResponseType.DataCorrupt.value:
            self.server.send(self.conn, self.addr, self.last_data_type_sent, self.last_buffer_sent)
            
    def handle_string(self, buffer):
        print(buffer.decode('utf-8'))
        self.server.send_response(self.conn, self.addr, NetworkResponseType.AllGood) 
        
    def handle_network_data(self, conn, addr, data_type, buffer):
        self.conn = conn
        self.addr = addr
    
        if data_type == NetworkDataType.String.value:
            self.handle_string(buffer)
        elif data_type == NetworkDataType.Response.value:
            self.handle_response(buffer)
        

class Server:
    def __init__(self, TCP_IP, PORT):
        print("[STARTING] Server is starting")
        self.TCP_IP = TCP_IP
        self.PORT   = PORT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.TCP_IP, self.PORT))
        self.data_handler = NetworkDataHandler(self)
        self.handlers = []
        
    def send(self, conn, addr, data_type, buffer):
        conn.send(struct.pack('>h',data_type.value))
        conn.send(struct.pack('>q',len(buffer)))
        conn.send(buffer)
        
    def send_response(self, conn, addr, response):
        send(conn, addr, NetworkDataType.Response, struct.pack('>h',response.value))
        
    def listen(self, conn, addr):

        input_buff = []
        
        data_type  = struct.unpack('>h', conn.recv(2))[0]
        chunk_size = struct.unpack('>i', conn.recv(4))[0]
        chunks     = struct.unpack('>i', conn.recv(4))[0]
        residual   = struct.unpack('>i', conn.recv(4))[0]
        
        for i in range(chunks):
            input_buff.append(conn.recv(chunk_size))

        input_buff.append(conn.recv(residual))
        input_buff = b''.join(input_buff)
        
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
