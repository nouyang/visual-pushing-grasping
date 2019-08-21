import socket

tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_host_ip = "10.75.15.91"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002

tcp_socket.connect((tcp_host_ip, tcp_port))
tcp_command = '''
textmsg("hi")
def prog():
# movej([0.000000, -0.785398, 1.570796, -1.377065, -1.570796, 3.14159], a=8.000000, v=15.000000, t=0.0, r=0.0000)
    # set_tool_voltage(0)
    # sleep(2.0)
    # set_digital_out(8, False)
    # set_digital_out(9, False)
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
    # set_digital_out(9, False)
    # sleep(1.0)
    # count = 0
    # while True:
        # textmsg(count)
        # set_digital_out(8, False)
        # sleep(1.0)
        set_digital_out(8, True)
        sleep(1.5)
        # count = count + 1
    # end
movej([0.000000, -0.785398, 1.570796, -1.377065, -1.570796, 3.14159], a=8.000000, v=15.000000, t=0.0, r=0.090000)
movej([0.000000, -1.148427, 0.263545, -2.270673, -1.570796, 3.14159], a=25.000000, v=3.200000, t=0.0, r=0.600000)
set_digital_out(8,False)
movej([0.000000, -0.785398, 1.570796, -1.377065, -1.570796, 3.14159], a=8.000000, v=15.000000, t=0.0, r=0.0)
end

end
'''
# tcp_command += " set_digital_out(8,False)\n"
# false is open
# tcp_command += "end\n"

tcp_socket.send(str.encode(tcp_command))
tcp_socket.close()


# tcp_socket.close()
print('done')
