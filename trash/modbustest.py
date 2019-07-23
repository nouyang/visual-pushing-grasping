from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import urx
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.WARN)

tcp_host_ip = "10.75.15.91"


# Connect to the IP adresse of the robot controller
client = ModbusClient(tcp_host_ip, port=502)

client.connect()

# Writting desired x position in register 128
rq = client.write_register(128, 100)
rr = client.read_holding_registers(128, 1)
assert(rq.function_code < 0x80)     # test that we are not an error

print('Hey I read a register! 128', rr.registers[0])

# Writting desired y position in register 129
rq = client.write_register(129, 40)
rr = client.read_holding_registers(129, 1)
assert(rq.function_code < 0x80)     # test that we are not an error

print rr.registers[0]

client.close()


tcp_host_ip = "10.75.15.91"
rob = urx.Robot(tcp_host_ip)
robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
robotiqgrip.gripper_action(rr.registers[0])

rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(0.5, (0, 0, 0))

rob.send_message("hi")
rob.send_message("rq_get_position()")
data = rob.secmon.get_all_data(wait=False)

rob.send_program('pose_x:=read_port_register(128)')
rob.send_program('pose_x')

rob.close()
