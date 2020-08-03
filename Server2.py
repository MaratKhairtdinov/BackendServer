import socket

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.0.101"#socket.gethostname()
port = 5555
print(host)
print(port)
serversocket.bind((host,port))

serversocket.listen(5)

print('server started and listening')
while 1:
    (clientsocket, address) = serversocket.accept()
    print("Connection found")
    
    #Decode ASCII
    #data = clientsocket.recv(1024).decode('ascii')
    
    #Decode Bytes
    data = clientsocket.recv(1024).decode("utf-8")
    print(data)
 
    if data == "Ping":
        print ("Unity Sent: " + str(data))
        #clientsocket.send("Pong").encode("utf-8")
    else:
        print(data)
    print("closed socket")
    clientsocket.close()
