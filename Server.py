import socket
import threading
import time
import struct

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
    print(f"Client sent: {message}")

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

def receive_PCD(conn, message_length):    
    points = ""
    counter = 0
    intChunks = int.from_bytes(conn.recv(8), byteorder = 'big', signed = False);
    modulo = int.from_bytes(conn.recv(8), byteorder = 'big', signed    = False);
    print("Starting to receive the PointCloud")
    for i in range(intChunks):        
        for j in range(100):
            x = struct.unpack('>d', conn.recv(8))[0]
            y = struct.unpack('>d', conn.recv(8))[0]
            z = struct.unpack('>d', conn.recv(8))[0]
            normal_x = struct.unpack('>d', conn.recv(8))[0]
            normal_y = struct.unpack('>d', conn.recv(8))[0]
            normal_z = struct.unpack('>d', conn.recv(8))[0]            
            points += f"\n{x} {z} {y} {normal_x} {normal_y} {normal_z}"
        print(f"Chunk # {i} received")
        time.sleep(0.1)
    for i in range(modulo):
        x = struct.unpack('>d', conn.recv(8))[0]
        y = struct.unpack('>d', conn.recv(8))[0]
        z = struct.unpack('>d', conn.recv(8))[0]
        normal_x = struct.unpack('>d', conn.recv(8))[0]
        normal_y = struct.unpack('>d', conn.recv(8))[0]
        normal_z = struct.unpack('>d', conn.recv(8))[0]
        points += f"\n{x} {z} {y} {normal_x} {normal_y} {normal_z}"
    size = intChunks*100+modulo
    write_PCD(points, size)
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



def send_message(message, conn):
    #responseBuff = f"Message received, Type: [{msg_type}]"
    conn.send(str(len(message)).ljust(64,' ').encode(FORMAT))
    conn.send(message.encode(FORMAT))
    print(f"{message} sent back to the client")



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

