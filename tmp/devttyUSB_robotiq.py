# https://blog.robotiq.com/controlling-the-robotiq-2f-gripper-with-modbus-commands-in-python

import serial
import time
import binascii

ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
counter = 0

while counter < 1:
    counter = counter + 1
    ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
    data_raw = ser.readline()
    print('Raw data', data_raw)
    data = binascii.hexlify(data_raw)
    print("Response 1 ", data)
    time.sleep(0.01)

    ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
    data_raw = ser.readline()
    print('Raw data', data_raw)
    data = binascii.hexlify(data_raw)
    print("Response 2 ", data)
    time.sleep(.2)

while(True):
    print("Close gripper")
    ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
    data_raw = ser.readline()
    print('Raw data', data_raw)
    data = binascii.hexlify(data_raw)
    print("Data Response 3 ", data)
    time.sleep(0.2)
    # closing_force = '\xFF' # 255
    clear_rACT = "\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30"

    print("!----- Read status")
    ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
    data_raw = ser.readline()
    print('Raw data', data_raw)
    data = binascii.hexlify(data_raw)
    print("Response 4 ", data)
    position = data[14:16]
    print("!--Position ", int(position, 16))
    time.sleep(.5)
    # Example response:
    # ('Raw data', '\t\x03\x06\xf9\x00\t\xff\xe6\x00+\x10')
    # ('Response 4 ', '090306f90009ffe6002b10')
    # ('Position ', 230 """

    print("Open gripper")
    open_cmd = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19"
    ser.write(open_cmd)
    print('open_cmd', open_cmd)

    data_raw = ser.readline()
    # print('Raw data', data_raw)
    # data = binascii.hexlify(data_raw)
    # print("Response 4 ", data)
    time.sleep(0.2)

    print("!----- Read status")
    ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
    data_raw = ser.readline()
    print('Raw data', data_raw)
    data = binascii.hexlify(data_raw)
    print("Response 4 ", data)
    position = data[14:16]
    print("!-- Position ", int(position, 16))
    time.sleep(.5)

    # ser.write(
    # "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
    """
    closing_force = '\xBB'  # 187
    close_cmd = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF" + \
        closing_force + "\x42\x29"
    print('close_cmd', close_cmd)
    ser.write(close_cmd)
    time.sleep(.5)
    """


# ----------------------------------

data_raw = ser.readline()
data = binascii.hexlify(data_raw)
# Example of response if the grip is not completed: 09 03 06    39 00 00 FF 0E 0A F7 8B
# Example of response if the grip is completed: 09 03 06        B9 00 00 FF BD 00 1D 7C
# BD = 188, aka position = 188/255

# 255 = fully closed

self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

self.tcp_socket.send(str.encode(tcp_command))
tcp_command = "movel(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0)\n" % (increment_position[0], increment_position[1],
                                                                   increment_position[2], tool_orientation[0], tool_orientation[1], tool_orientation[2], self.tool_acc, self.tool_vel)
self.tcp_socket.send(str.encode(tcp_command))

time_start = time.time()
tcp_state_data = self.tcp_socket.recv(2048)
