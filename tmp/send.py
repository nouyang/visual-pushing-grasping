import socket
import time
HOST = "10.75.15.91"    # The remote host
PORT = 30002              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print("Starting Program")
f = open("./rg2_new.script", "r")
s.send(f.read() + "/n")
data = s.recv(1024)
s.close()
print ("Closed connection and received data")
