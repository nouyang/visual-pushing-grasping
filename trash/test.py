# Echo client program
import socket
import binascii

HOST = "10.75.15.91"
PORT = 30002  # The same port as used by the server
# PORT = 63352

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

s.send("set_digital_out(2,True)" + "\n")
# data = s.recv(1024)
# print ("Received", repr(data))

# msg = "socket_get_var(POS)"
msg = "rq_current_pos()"
s.send(msg + "\n")
data_raw = s.recv(1024)
print ("Received", repr(data_raw))
s.close()

# print("!----- Read status")
# ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
# data_raw = ser.readline()
# print('Raw data', data_raw)
data = binascii.hexlify(data_raw)
print('data',  data)
# print(data_raw.decode("hex"))
msg = data_raw.encode("hex")
print('data', msg)
position = data[14:16]
print("!-- Position ", int(position, 16))
