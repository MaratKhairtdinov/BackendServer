import socket
import threading
import time

HEADER = 64
TCP_IP = '192.168.0.101'
TCP_PORT = 10000
BUFFER_SIZE = 1024
FORMAT = 'utf-8'
DISCONNECT_MSG = "!DISCONNECT!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))



def write_PCD(msg):
    file = open("ReceivedPointcloud.pcd","w")
    file.write(msg)
    file.close()
    print("[PCD_RECEIVED]")

def send_message(message, conn):
    #responseBuff = f"Message received, Type: [{msg_type}]"
    conn.send(str(len(message)).ljust(64,' ').encode(FORMAT))
    conn.send(message.encode(FORMAT))
    print(f"{message} sent back to the client")
    
    

def handle_client(conn, addr):
    print (f"[NEW CONNECTION] {addr} connected.")
    connected = True
    counter = 0
    while connected:
        counter += 1
        msg_length = conn.recv(HEADER).decode(FORMAT)        
        if msg_length:
            msg_length = int(msg_length)
            msg_type = conn.recv(msg_length).decode(FORMAT)        
            msg_length = conn.recv(HEADER).decode(FORMAT)        
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)        
            if msg_type == "PCD_FILE":
                write_PCD(msg)
            elif msg_type == DISCONNECT_MSG:
                connected = False                
            else:
                print(f"""[{addr}] sent: "{msg}" """)
            #time.sleep(1)
            send_message("Server replied", conn)
            
            #send_message(msg, conn)
            
            
        
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

