import socket
import struct
#
data_str = b'x00\x00\x007\x14\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x03\tURControl\x03\n\x00\x00\x00\x00\x00\x00\x00\x0003-05-2019, 06: 39: 00\x00\x00\x00\x18\x14\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x0c\x00\x00\x01\xfe\x00\x00\x01\x03\x01'
# tcp_state_data = self.tcp_socket.recv(2048)
data_str = b'\x00\x00\x007\x14\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x03\tURControl\x03\n\x00\x00\x00\x00\x00\x00\x00\x0003-05-2019, 06:39:00\x00\x00\x00\x18\x14\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x0c\x00\x00\x01\xfe\x00\x00\x01\x03\x01\x00\x00\x00\xb5\x18?\xc3333333@9\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00@\x08\x00\x00\x00\x00\x00\x00?\xc9\x99\x99\x99\x99\x99\x9a?\xc9\x99\x99\x99\x99\x99\x9a?\xc9\x99\x99\x99\x99\x99\x9a?\xc9\x99\x99\x99\x99\x99\x9a?\xc9\x99\x99\x99\x99\x99\x9a?\xc9\x99\x99\x99\x99\x99\x9a?\xaa\xce\xe9\xf7C\xdd\x94?\xaa\xce\xe9\xf7C\xdd\x94?\xaa\xce\xe9\xf7C\xdd\x94?\xaa\xce\xe9\xf7C\xdd\x94?\xaa\xce\xe9\xf7C\xdd\x94?\xaa\xce\xe9\xf7C\xdd\x94\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00?\x91\xdfFw\x07\xc9M?PbM\xd2\xf1\xa9\xfc?\x84z\xe1G\xae\x14{'


def parse_tcp_state_data(tcp_socket, subpackage):
    state_data = tcp_socket.recv(1024)
    # Read package header

    data_bytes = bytearray()
    data_bytes.extend(state_data)
    data_length = struct.unpack("!iB", data_bytes[0:5])[0]
    robot_message_type = struct.unpack("!iB", data_bytes[0:5])[1]

    # print('robot message type', robot_message_type)

    while (robot_message_type != 16):
        print('keep trying')
        state_data = tcp_socket.recv(1024)
        data_bytes = bytearray()
        data_bytes.extend(state_data)
        data_length = struct.unpack("!iB", data_bytes[0:5])[0]
        robot_message_type = struct.unpack("!iB", data_bytes[0:5])[1]

    byte_idx = 5

    # Parse sub-packages
    subpackage_types = {
        'joint_data': 1, 'cartesian_info': 4, 'force_mode_data': 7, 'tool_data': 2}
    while byte_idx < data_length:
        # package_length = int.from_bytes(data_bytes[byte_idx:(byte_idx+4)], byteorder='big', signed=False)
        package_length = struct.unpack(
            "!i", data_bytes[byte_idx:(byte_idx+4)])[0]
        byte_idx += 4
        package_idx = data_bytes[byte_idx]
        if package_idx == subpackage_types[subpackage]:
            byte_idx += 1
            break
        byte_idx += package_length - 4
    tcp_socket.close()

    def parse_joint_data(data_bytes, byte_idx):
        actual_joint_positions = [0, 0, 0, 0, 0, 0]
        target_joint_positions = [0, 0, 0, 0, 0, 0]
        for joint_idx in range(6):
            actual_joint_positions[joint_idx] = struct.unpack(
                '!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            target_joint_positions[joint_idx] = struct.unpack(
                '!d', data_bytes[(byte_idx+8):(byte_idx+16)])[0]
            byte_idx += 41
        # DEBUG:
        # print('joint pos', actual_joint_positions)
        return actual_joint_positions

    def parse_cartesian_info(data_bytes, byte_idx):
        actual_tool_pose = [0, 0, 0, 0, 0, 0]
        for pose_value_idx in range(6):
            actual_tool_pose[pose_value_idx] = struct.unpack(
                '!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
            byte_idx += 8
        # print('tool pos', actual_tool_pose)
        return actual_tool_pose

    def parse_tool_data(data_bytes, byte_idx):
        byte_idx += 2
        tool_analog_input2 = struct.unpack(
            '!d', data_bytes[(byte_idx+0):(byte_idx+8)])[0]
        return tool_analog_input2

    parse_functions = {'joint_data': parse_joint_data,
                       'cartesian_info': parse_cartesian_info, 'tool_data': parse_tool_data}
    return parse_functions[subpackage](data_bytes, byte_idx)


tcp_host_ip = "10.75.15.91"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.connect((tcp_host_ip, tcp_port))

# printout = parse_tcp_state_data(tcp_socket, 'tool_data')
printout = parse_tcp_state_data(tcp_socket, 'cartesian_info')
print(repr(printout))
