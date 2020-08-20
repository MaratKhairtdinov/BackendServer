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

def receive_string(conn, message_length):
    message = conn.recv(message_length).decode(FORMAT)
    send_message(message, conn)
    print(f"Client sent: {message}")

def send_message(message, conn):
    #responseBuff = f"Message received, Type: [{msg_type}]"
    conn.send(str(len(message)).ljust(64,' ').encode(FORMAT))
    conn.send(message.encode(FORMAT))
    print(f"{message} sent back to the client")
        
def handle_client(conn, addr):
    print (f"[NEW CONNECTION] {addr} connected.")
    pointcloud = o3d.geometry.PointCloud()
    connected = True    
    while connected:
        msg_type = struct.unpack('>h', conn.recv(2))  [0]
        msg_length = struct.unpack('>q', conn.recv(8))[0]
        if msg_type == 2:
            receive_string(conn, msg_length)
        elif msg_type == 3:
            receive_PCD(conn, msg_length)        
            
        
def start():
    s.listen()
    print("[LISTENING] Server is listening on {SERVER}")
    while True:
        print(f"[ACTIVE CONNECTIONS]{threading.activeCount()-1}")
        conn, addr = s.accept()
        thread = threading.Thread(target = handle_client, args = (conn, addr))
        thread.start()
        

print ("[STATING] server is starting...")
start()

