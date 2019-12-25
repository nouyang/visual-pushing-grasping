import socket

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_host_ip = "10.75.15.199"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002

tcp_socket.connect((tcp_host_ip, tcp_port))
tcp_command = '''
textmsg("hi")
def andy_throw():
        set_digital_out(9, False)
        set_digital_out(8, True)
        sleep(2.)
        movej([0.000000, -0.7854, 1.571, -1.726, -1.570796, 3.14159], a=8.00, v=3.00, t=0.0, r=0.09)
        movej([0.000000, -1.0088, 0.613, -2.522, -1.570796, 3.14159], a=28.00, v=4.20, t=0.0, r=0.4)
        set_digital_out(8,False)
        movej([0.000000, -0.7854, 1.571, -1.726, -1.570796, 3.14159], a=3.00, v=3.00, t=0.0, r=0.00)
end
'''
pretoss = [0.00, -140.5, -23.72, -217.5, -90.0, 180.0]
posttoss = [0.00, -58.1, 14.22, -208.4, -90.0, 180.0]

prettoss = [0., -2.452, -0.414, -3.796, -1.571,  3.142]
posttoss = [0., -1.014,  0.248, -3.637, -1.571,  3.142]

overhand_command = '''
textmsg("overhand")
def overHand():
        set_digital_out(9, False)
        sleep(0.5)
        set_digital_out(8, True)
        sleep(2.)
        movej([0., -2.452, -0.414, -3.796, -1.571,  3.142], a=8.00, v=3.00, t=0.0, r=0.09)
        movej([0., -1.514,  1.148, -3.637, -1.571,  3.142], a=25.00, v=3.20, t=0.0, r=0.5)
        set_digital_out(8,False)
        movej([0., -2.452, -0.414, -3.796, -1.571,  3.142], a=8.00, v=3.00, t=0.0, r=0.00)
end
'''

tcp_command += "end\n"
tcp_socket.send(str.encode(tcp_command))
tcp_socket.close()

# tcp_socket.close()
print('done')

# movej([0.000000, -0.785398, 1.570796, -1.377065, -1.570796, 3.14159], a=8.000000, v=15.000000, t=0.0, r=0.0000)
# set_tool_voltage(0)
# sleep(2.0)
# # set_digital_out(8, False)
# # set_digital_out(9, False)
# set_tool_voltage(24)
# sleep(2.0)
# timeout = 0
# while get_digital_in(9) == False:
# timeout = timeout+1
# # sleep(0.008)
# sleep(0.005)
# if timeout > 800:
# # wait at most 5 secs
# textmsg("breaking")
# break
# end
# end
# sleep(1.0)
# count = 0
# while True:
# textmsg(count)
# set_digital_out(8, False)
# sleep(1.0)
# count = count + 1
# end
