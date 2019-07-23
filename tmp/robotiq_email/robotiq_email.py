import binascii
import socket
# IP of the robot. PC and robot have to be in same sub network. In this case 10.20.1.xxx
HOST = "10.75.15.91"
# The same port as used by the server. Here primary interface.
PORT = 30001
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    # with open("hello.script") as script:
    with open("BasicPosRq.script") as script:
        command = script.read()
        command = command+'\n'
        s.send(command.encode('ascii'))
        # s.send(command.encode('utf-8'))
        data_raw = s.recv(2048)
        data = binascii.hexlify(data_raw)
        s.close()
        print ("Received", repr(data_raw))
        print ("Received", data)
