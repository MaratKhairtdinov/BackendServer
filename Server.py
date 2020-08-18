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
    send_message(message, connzz)
    print(f"Client sent: {message}")

def send_message(message, conn):
    #responseBuff = f"Message received, Type: [{msg_type}]"
    conn.send(str(len(message)).ljust(64,' ').encode(FORMAT))
    conn.send(message.encode(FORMAT))
    print(f"{message} sent back to the client")

def visualize_PCD(name):
    point_cloud = o3d.io.read_point_cloud(f'{name}')
    point_cloud.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius = 0.5, max_nn = 30))
    o3d.visualization.draw_geometries([point_cloud], top = 30, left = 0, point_show_normal=True)

def write_PCD(points_list, number):
    header = f""" VERSION .7
FIELDS x y z normal_x normal_y normal_z
SIZE 4 4 4 4 4 4
TYPE F F F F F F
COUNT 1 1 1 1 1 1
WIDTH {number}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {number}
DATA ascii"""
    
    header += points_list
    name = 'ReceivedPointCloud.pcd'                                  
    file = open(name,"w")
    file.write(header)
    file.close()
    print("[PCD_WRITTEN_INTO_FILE]")
    #visualize_PCD(name)
    



def receive_PCD(conn, message_length):   
    points = ""
    errorLog = ""
    chunks = message_length
    step = int.from_bytes(conn.recv(HEADER), byteorder='big', signed=False)
    counter = 0;
    for i in range(chunks):
        print(f"Chunk #{i}")
        for j in range(step):
            #print(f"Vertex #{i*step+j}")
            try:
                x = struct.unpack('>d',conn.recv(8))[0]
                z = struct.unpack('>d',conn.recv(8))[0]
                y = struct.unpack('>d',conn.recv(8))[0]
                n_x = struct.unpack('>d',conn.recv(8))[0]
                n_z = struct.unpack('>d',conn.recv(8))[0]
                n_y = struct.unpack('>d',conn.recv(8))[0]
                points+=f"\n{x} {y} {z} {n_x} {n_y} {n_z}"
                counter+=1;
            except:
                errorLog+=f"\nChunk #{i} is corrupt"
    send_message(errorLog, conn)
    if len(errorLog)==0:        
        write_PCD(points, counter)
        send_message(f"[SERVER: POINTCLOUD RECEIVED]", conn)
    else:        
        print(errorLog)
            
        
def handle_client(conn, addr):
    print (f"[NEW CONNECTION] {addr} connected.")
    pointcloud = o3d.geometry.PointCloud()
    connected = True    
    while connected:
        msg_type =   int.from_bytes(conn.recv(HEADER), byteorder = 'big', signed = False)
        msg_length = int.from_bytes(conn.recv(HEADER), byteorder = 'big', signed = False)
        if msg_type == 1:
            receive_string(conn, msg_length)
        elif msg_type == 2:
            receive_PCD(conn, msg_length)        
            
        
def start():
    s.listen()
    print("[LISTENING] Server is listening on {SERVER}")
    while True:
        print(f"[ACTIVE CONNECTIONS]{threading.activeCount()-1}")
        conn, addr = s.accept()
        thread = threading.Thread(target = handle_client, args = (conn, addr))
        thread.start()
        #print(f"[ACTIVE CONNECTIONS]{threading.activeCount()-1}")

print ("[STATING] server is starting...")
start()

