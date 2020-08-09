import socket
import threading
import time
import struct
import open3d as o3d
import numpy as np
import copy

HEADER = 8
TCP_IP = '192.168.0.102'
TCP_PORT = 10000
BUFFER_SIZE = 1024
FORMAT = 'utf-8'
DISCONNECT_MSG = "!DISCONNECT!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))

def receive_string(conn, message_length):
    message = conn.recv(message_length).decode(FORMAT)
    print(f"Client sent: {message}")

def send_message(message, conn):
    #responseBuff = f"Message received, Type: [{msg_type}]"
    conn.send(str(len(message)).ljust(64,' ').encode(FORMAT))
    conn.send(message.encode(FORMAT))
    print(f"{message} sent back to the client")

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
    header+=points_list
    file = open("ReceivedPointcloud.pcd","w")
    file.write(header)
    file.close()
    print("[PCD_RECEIVED]")
    point_cloud = o3d.io.read_point_cloud(r'C:\Users\Marat\Documents\Thesis\PythonServer\ReceivedPointcloud.pcd')
    o3d.visualization.draw_geometries([point_cloud], top = 30, left = 0, point_show_normal=False)

def receive_PCD(conn, message_length):    
    points = ""
    errorLog="Error Log:"
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
    send_message("[SERVER: POINTCLOUD RECEIVED]", conn)
    print(errorLog)
    write_PCD(points, counter)
    
    
            
    """points = ""
    #vertices = int.from_bytes(conn.recv(8),byteorder = 'big', signed=False)
    counter = 0
    for i in range(message_length):
        counter += 1
        #if(i%500==0):
        #    print(f"Chunk #{i}")
        size = int.from_bytes(conn.recv(8),byteorder = 'big', signed=False)
        print(f"cChunk #{i};\t{size} characters are waited")
        line = conn.recv(size).decode(FORMAT)
        points += line
    write_PCD(points, i)
"""
    """
    for i in range(message_length):        
        print(i)
        #time.sleep(.000001)
        x_bytes = conn.recv(8)
        y_bytes = conn.recv(8)
        z_bytes = conn.recv(8)
        x_n_bytes = conn.recv(8)
        y_n_bytes = conn.recv(8)
        z_n_bytes = conn.recv(8)
        try:
            x = struct.unpack('>d', x_bytes)[0]
            y = struct.unpack('>d', y_bytes)[0]
            z = struct.unpack('>d', z_bytes)[0]
            normal_x = struct.unpack('>d', x_n_bytes)[0]
            normal_y = struct.unpack('>d', y_n_bytes)[0]
            normal_z = struct.unpack('>d', z_n_bytes)[0]
            counter += 1
            points += f"\n{x}    {z}    {y}    {normal_x}    {normal_y}    {normal_z}"
        except:
            print( f"X: {len(x_bytes)}Y: {len(y_bytes)} Z: {len(z_bytes)}\tX_n: {len(x_n_bytes)} Y_n: {len(y_n_bytes)} Z_n: {len(z_n_bytes)}" )        
    write_PCD(points, counter)
    """
    







def handle_client(conn, addr):
    print (f"[NEW CONNECTION] {addr} connected.")
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

