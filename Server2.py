import socket
import threading
import time
import struct
import open3d as o3d
import numpy as np
import copy

HEADER = 8
TCP_IP = '192.168.0.100'
TCP_PORT = 10000
BUFFER_SIZE = 1024
FORMAT = 'utf-8'
DISCONNECT_MSG = "!DISCONNECT!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))

class NetworkDataHandler:
    def __init__(self, client)
        self.client = client

class Client:
    def __init__(self, TCP_IP, TCP_PORT, FORMAT):
        self.TCP_IP = TCP_IP
        self.TCP_PORT = TCP_PORT
        self.FORMAT = FORMAT
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        self.network_data_handler = NetworkDataHandler(self)
    def start_listening():
        self.s.listen()
        print("[Listening] Server is listening on {SERVER}"
    
    

















