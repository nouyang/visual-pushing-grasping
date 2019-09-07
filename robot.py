import socket
import select
import struct
import time
import os
import numpy as np
import utils
import serial
import binascii

from pyUR import PyUR

from simulation import vrep
from real.camera import Camera


class Robot(object):
    def __init__(self, is_sim, obj_mesh_dir, num_obj, workspace_limits,
                 tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                 is_testing, test_preset_cases, test_preset_file,
                 home_joint_config=None):

        self.orig = False

        # If in simulation...
        if self.is_sim:
            pass
        # If in real-settings...
        else:
            # self.gripper = Robotiq_Two_Finger_Gripper(self.r)

            # Default joint speed configuration
            # self.joint_acc = 8 # Safe: 1.4
            # self.joint_vel = 3 # Safe: 1.05
            self.joint_acc = 1.0  # Safe when set 30% spe71ed on pendant
            self.joint_vel = 0.7

            # Connect to robot client
            # self.tcp_host_ip = tcp_host_ip
            # self.tcp_port = tcp_port

            # NOTE: port is assumed to be 30002
            self.workspace_limits = workspace_limits

            self.r = PyUR(tcp_host_ip, self.joint_vel,
                          self.joint_acc, home_joint_config=home_joint_config,
                          workspace_limits=workspace_limits)

            # Move robot to home pose
            self.r.go_home()
            # self.r.activate_gripper()
            # self.r.close_gripper()
            # self.r.open_gripper()

            # Fetch RGB-D data from RealSense camera
            self.camera = Camera()
            self.cam_intrinsics = self.camera.intrinsics

            # Load camera pose (from running calibrate.py), intrinsics and depth scale
            self.cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
            self.cam_depth_scale = np.loadtxt(
                "real/camera_depth_scale.txt", delimiter=" ")

    def get_camera_data(self):

        if self.is_sim:
            pass

        else:
            # Get color and depth image from ROS service
            color_img, depth_img = self.camera.get_data()
            # color_img = self.camera.color_data.copy()
            # depth_img = self.camera.depth_data.copy()

        return color_img, depth_img

    ''' NOT NEEDED ANYMORE
    def grasp_object(self, position, orientation):
        # TODO: right now not batching into single urscript function; could do
        # in future
        print('grasp prediction z:', position[2])

        self.r.open_gripper()
        # NOTE: Hardcoded values as per vpg code
        orientation[2] = 0
        self.r.move_to([position[0], position[1], position[2] + 0.1],
                       orientation, vel=self.joint_vel*0.5,
                       acc=self.joint_acc*0.5)
        # then slowly move down
        print('position', position, 'orientation', orientation)
        self.r.move_to(position, orientation)
        # vel=0.5*self.joint_vel, acc=0.1*self.joint_acc)
        # and grasp it
        self.r.close_gripper()
    '''

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' %
              (position[0], position[1], position[2]))

        if self.is_sim:
            pass

        else:
            grasp_orientation = [1.0, 0.0]
            # Compute tool orientation from heightmap rotation angle
            if heightmap_rotation_angle > np.pi:
                heightmap_rotation_angle = heightmap_rotation_angle - 2*np.pi
            tool_rotation_angle = heightmap_rotation_angle/2
            # NOTE: this is a weird way of saying [np.cos, np.sin]
            tool_orientation = np.asarray([
                grasp_orientation[0]*np.cos(tool_rotation_angle) -
                grasp_orientation[1]*np.sin(tool_rotation_angle),
                grasp_orientation[0]*np.sin(tool_rotation_angle) +
                grasp_orientation[1]*np.cos(tool_rotation_angle),
                0.0]) * np.pi
            tool_orientation_angle = np.linalg.norm(tool_orientation)
            tool_orientation_axis = tool_orientation / tool_orientation_angle
            tool_orientation_rotm = utils.angle2rotm(
                tool_orientation_angle, tool_orientation_axis, point=None)[:3, :3]

            # Compute tilted tool orientation during dropping into bin
            tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4, 0, 0]))
            tilted_tool_orientation_rotm = np.dot(
                tilt_rotm, tool_orientation_rotm)
            tilted_tool_orientation_axis_angle = utils.rotm2angle(
                tilted_tool_orientation_rotm)
            tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(
                tilted_tool_orientation_axis_angle[1:4])

            position = np.asarray(position).copy()
            position[2] = max(position[2] - 0.05, workspace_limits[2][0])
            # NOTE: not needed anymore
            # self.grasp_object(position, tool_orientation)

            # Attempt grasp
            tool_orientation[2] = 0
            position[2] += 0.1
            above = {'type': p, 'pose': np.append(position, tool_orientation),
                     'acc': self.joint_acc * 0.5, 'vel': self.joint_vel * 0.5,
                     'rad': 0.09}
            position[2] -= 0.1
            down = {'type': p, 'pose': np.append(position, tool_orientation),
                    'acc': self.joint_acc * 0.1, 'vel': self.joint_vel * 0.1,
                    'rad': 0.00}
            # Block until robot reaches target tool position and gripper fingers have stopped moving
            self.r.combo_move([{'type': 'open'}, above, down, {'type': 'close'}],
                              wait=True)

            # TODO: change these values! -----
            prepick_position = [0, 0, 0]
            pretoss_position = [0, 0, 0]

            gripper_width = self.r.get_state('tool_data')
            gripper_open = gripper_width > 0.26

            if gripper_open:
                # We've grasped something. Move to pretoss position
                # blend in nice arc b/tw current pos and pretoss pos
                blend_radius = min(0.02,
                                   abs(pretoss_position[1] - position[1])/2 - 0.01)

                position[2] = pretoss_position[2]
                self.r.move_to(position, tool_orientation, radius=blend_radius)

                # Check if grip has slipped (width has changed). If not, success
                # TODO: this relies on gripper continuing to try to close
                if np.abs(self.r.get_state('tool_data') - gripper_width) < 0.1:
                    grasp_success = True
                else:
                    # failed to keep object in grasp, move up & to home to
                    # prepare to try again
                    position[2] += 0.1
                    self.r.move_to(position, tool_orientation,
                                   acc=self.joint_acc * 0.5,
                                   vel=self.joint_vel * 0.5,
                                   radius=0.09)
                    self.r.move_to(home_position, tool_orientation,
                                   acc=self.joint_acc * 0.5,
                                   vel=self.joint_vel * 0.5,
                                   radius=0.0)
                    grasp_success = False
                return grasp_success

    def throw(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: throw at (%f, %f, %f)' %
              (position[0], position[1], position[2]))
        pretoss_jconf = np.array([0.000000, -0.7854, 1.571, -1.726, -1.570796,
                                  3.14159])
        posttoss_jconf = np.array([0.000000, -1.0088, 0.613, -2.522, -1.570796,
                                   3.14159])

        pretoss = {'type': j, 'pose': pretoss_jconf,
                   'acc': 8.00, 'vel': 3.00, 'rad': 0.09}
        posttoss = {'type': j, 'pose': posttoss_jconf,
                    'acc': 28.00, 'vel': 4.20, 'rad': 0.4}
        back_to_pretoss = {'type': j, 'pose': pretoss_jconf,
                           'acc': 3.00, 'vel': 3.00, 'rad': 0.0}

        # Block until robot reaches target tool position and gripper fingers have stopped moving
        self.r.combo_move([{'type': 'close'}, pretoss, posttoss, {'type': 'open'},
                           back_to_pretoss], wait=True)


def restart_real(self):

    print('DEBUG: restarting real')

    # Compute tool orientation from heightmap rotation angle
    grasp_orientation = [1.0, 0.0]
    tool_rotation_angle = -np.pi/4
    tool_orientation = np.asarray([grasp_orientation[0]*np.cos(tool_rotation_angle) - grasp_orientation[1]*np.sin(
        tool_rotation_angle), grasp_orientation[0]*np.sin(tool_rotation_angle) + grasp_orientation[1]*np.cos(tool_rotation_angle), 0.0])*np.pi
    tool_orientation_angle = np.linalg.norm(tool_orientation)
    tool_orientation_axis = tool_orientation/tool_orientation_angle
    tool_orientation_rotm = utils.angle2rotm(
        tool_orientation_angle, tool_orientation_axis, point=None)[:3, :3]

    tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4, 0, 0]))
    tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
    tilted_tool_orientation_axis_angle = utils.rotm2angle(
        tilted_tool_orientation_rotm)
    tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(
        tilted_tool_orientation_axis_angle[1:4])

    def throw_sideways(self, is_sim=False):
        K = 9.
        acc, vel = 1.4 * K, K

        sideways_position = np.deg2rad(np.array(
            [-107, -105, 130, -92, -44, -30]) * 1.0)  # sideways # bent wrist 1 2
        # [-107, -105, 130, -85, -90, -30]) * 1.0  # sideways bent wrist
        # sideways_position = [-0.005, 0.351, 0.245, 3.0, 0.33, 0.00]
        # sideways_position = [-0.017, 0.309,
        # 0.218, 2.6, 0.30, -0.23]  # bent wrist
        sideways_move = {'type': 'j',
                         'pose': sideways_position,
                         # 'acc': None, 'vel': None, 'radius': 0.2}
                         'acc': acc, 'vel': vel, 'radius': 0.05}

        start_position = [0.350, 0.000, 0.250, 2.12, -2.21, -0.009]
        start_move = {'type': 'p',
                      'pose': start_position,
                      # 'acc': None, 'vel': None, 'radius': 0.2}
                      'acc': acc/3., 'vel': vel/3., 'radius': 0.1}

        curled_position = [0.350, 0.000, 0.250, 1.75, -1.80, -0.62]
        curled_move = {'type': 'p',
                       'pose': curled_position,
                       'acc': acc, 'vel': vel, 'radius': 0.001}

        throw_position = [0.597, 0.000, 0.640, 2.26, -2.35, 2.24]
        # throw_position = [0.567, 0.000, 0.580, 2.38, -2.37, 1.60]
        throw_move = {'type': 'p',
                      'pose': throw_position,
                      'acc': acc, 'vel': vel, 'radius': 0.200}

        home_position = np.array(start_position) + \
            np.array([0, 0, 0.070, 0, 0, 0])
        home_position = home_position.tolist()
        home_move = {'type': 'p',
                     'pose': home_position,
                     'acc': acc/3., 'vel': vel/3., 'radius': 0.015}

        gripper_open = {'type': 'open'}

        # NOTE: important
        throw_pose_list = [throw_move,  # throw_move,
                           gripper_open, home_move, start_move, sideways_move]
        # throw_pose_list = [start_move]

        # pose_list = [start_pose, middle_pose, end_pose, start_pose]
        self.combo_move(throw_pose_list, wait=True, is_sim=is_sim)

    def throw_overhand(self, wait=True, RG2=True):
        pass

    def throw_andy(self, wait=True, is_sim=False):
        default_jacc = 8.  # 8
        default_jvel = 15.0  # 10
        toss_jacc = 25  # 25.
        toss_jvel = 3.2  # 3.2
        pretoss_jconf = np.asarray(
            [0., -45., 90., -078.9, -90., 0.])*np.pi/180.0
        # np.asarray([0., -45., 90., -098.9, -90., 0.]) * \
        # np.pi/180.0  # per email
        posttoss_jconf = np.asarray([0., -065.8, 015.1, -130.1, -90., 0.]) * \
            np.pi/180.0
        # np.asarray([0., -057.8, 035.1, -142.1, -90., 0.]) * \
        # np.pi/180.0  # per email
        pretoss_blend_radius = 0.09  # TODO: this does get used at all?
        # toss_blend_radius = 0.7 # BLEND FAIL
        toss_blend_radius = 0.6
        # toss_blend_radius = 0.05

        tcp_msg = "def process():\n"
        tcp_msg += '    socket_open("127.0.0.1",63352,"gripper_socket")\n'
        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("SPE", 255,
                                                                   self.socket_name)
        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("FOR", 0,
                                                                   self.socket_name)
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=%f)\n' % \
            (pretoss_jconf[0], pretoss_jconf[1], pretoss_jconf[2], pretoss_jconf[3],
             pretoss_jconf[4], pretoss_jconf[5], default_jacc, default_jvel, pretoss_blend_radius)
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=%f)\n' % \
            (posttoss_jconf[0], posttoss_jconf[1], posttoss_jconf[2], posttoss_jconf[3],
             posttoss_jconf[4], posttoss_jconf[5], toss_jacc, toss_jvel, toss_blend_radius)
        # tcp_msg += '    set_digital_out(8,False)\n'  # for RG2 gripper

        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("POS", 0,
                                                                   self.socket_name)
        # tcp_msg += "    sync()\n"
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=0.0)\n' % \
            (pretoss_jconf[0], pretoss_jconf[1], pretoss_jconf[2], pretoss_jconf[3],
             pretoss_jconf[4], pretoss_jconf[5], default_jacc, default_jvel)
        tcp_msg += '    socket_close("gripper_socket")\n'
        tcp_msg += 'end\n'
        self.send_program(tcp_msg, is_sim=is_sim)

        # tcp_msg += self._format_move("movej", pretoss_jconf, default_jacc,
        # default_jvel, pretoss_blend_radius, time=0, prefix="") + "\n"
        # tcp_msg += self._format_move("movej", posttoss_jconf, toss_jacc,
        # toss_jvel, toss_blend_radius, time=0, prefix="") + "\n"
        # tcp_msg += self._format_move("movej", pretoss_jconf, default_jacc,
        # default_jvel, radius=0, time=0, prefix="") + "\n"

        if wait:
            joint_flag = True
            self._wait_for_move(target=pretoss_jconf,
                                threshold=self.pose_tolerance, joints=joint_flag)
            return self.get_state('cartesian_info')

        print('done with toss')
        # self.send_program(prog, is_sim=is_sim)

    def throw(self, is_sim=False):
        self.close_gripper()
        # currently hard coded positions
        # acc, vel = 1.4, 1.05 # Safe
        # acc, vel = 8, 3 # Default
        # acc, vel = 15, 10
        # K = 8.5
        K = 2
        acc, vel = 1.4 * K, K
        start_position = [0.350, 0.000, 0.250, 2.12, -2.21, -0.009]
        start_move = {'type': 'p',
                      'pose': start_position,
                      # 'acc': None, 'vel': None, 'radius': 0.2}
                      'acc': acc, 'vel': vel, 'radius': 0.001}

        # curled_position = [0.350, 0.000, 0.250, 1.75, -1.80, -0.62]
        # curled_move = {'type': 'p',
        # 'pose': curled_position,
        # 'acc': acc, 'vel': vel, 'radius': 0.000}

        # throw_position = [0.597, 0.000, 0.640, 2.26, -2.35, 2.24]
        throw_position = [0.550, 0.000, 0.550, 2.26, -2.35, 2.24]
        # throw_position = [0.567, 0.000, 0.580, 2.38, -2.37, 1.60]
        throw_move = {'type': 'p',
                      'pose': throw_position,
                      'acc': acc, 'vel': vel, 'radius': 0.150}

        throw2_position = np.array(throw_position) + \
            np.array([0.050, 0.000, 0.105, 0, 0, 0])
        throw2_position = throw2_position.tolist()
        throw2_move = {'type': 'p',
                       'pose': throw2_position,
                       'acc': acc, 'vel': vel, 'radius': 0.01}  # max =0.05*1.4
        # return_position = [0.590, 0.000, 0.620, 2.38, -2.37, 1.60]
        # return_move = {'type': 'p',
        # 'pose': return_position,
        # 'acc': 3.5, 'vel': 3.5, 'radius': 0.100}

        home_position = np.array(start_position) + \
            np.array([0.010, 0, 0.210, 0, 0, 0])
        home_position = home_position.tolist()
        home_move = {'type': 'p',
                     'pose': home_position,
                     'acc': 1.0, 'vel': 1.0, 'radius': 0.001}

        gripper_open = {'type': 'open'}

        # throw_pose_list = [curled_move, throw_move,  # throw_move,
        throw_pose_list = [throw_move, gripper_open, throw2_move,
                           home_move, start_move]
        # throw_pose_list = [start_move]

        self.combo_move(throw_pose_list, wait=True, is_sim=is_sim)

        '''
        # Pre-compute blend radius
        # blend_radius = min(abs(bin_position[1] - position[1])/2 - 0.01, 0.2)
        # tcp_command += "movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=%f)\n" % \
        # (position[0], position[1], bin_position[2],
        # tool_orientation[0], tool_orientation[1], 0.0,
        # self.joint_acc, self.joint_vel, blend_radius)
        '''

    def throw_sideways(self, is_sim=False):
        K = 9.
        acc, vel = 1.4 * K, K

        sideways_position = np.deg2rad(np.array(
            [-107, -105, 130, -92, -44, -30]) * 1.0)  # sideways # bent wrist 1 2
        # [-107, -105, 130, -85, -90, -30]) * 1.0  # sideways bent wrist
        # sideways_position = [-0.005, 0.351, 0.245, 3.0, 0.33, 0.00]
        # sideways_position = [-0.017, 0.309,
        # 0.218, 2.6, 0.30, -0.23]  # bent wrist
        sideways_move = {'type': 'j',
                         'pose': sideways_position,
                         # 'acc': None, 'vel': None, 'radius': 0.2}
                         'acc': acc, 'vel': vel, 'radius': 0.05}

        start_position = [0.350, 0.000, 0.250, 2.12, -2.21, -0.009]
        start_move = {'type': 'p',
                      'pose': start_position,
                      # 'acc': None, 'vel': None, 'radius': 0.2}
                      'acc': acc/3., 'vel': vel/3., 'radius': 0.1}

        curled_position = [0.350, 0.000, 0.250, 1.75, -1.80, -0.62]
        curled_move = {'type': 'p',
                       'pose': curled_position,
                       'acc': acc, 'vel': vel, 'radius': 0.001}

        throw_position = [0.597, 0.000, 0.640, 2.26, -2.35, 2.24]
        # throw_position = [0.567, 0.000, 0.580, 2.38, -2.37, 1.60]
        throw_move = {'type': 'p',
                      'pose': throw_position,
                      'acc': acc, 'vel': vel, 'radius': 0.200}

        home_position = np.array(start_position) + \
            np.array([0, 0, 0.070, 0, 0, 0])
        home_position = home_position.tolist()
        home_move = {'type': 'p',
                     'pose': home_position,
                     'acc': acc/3., 'vel': vel/3., 'radius': 0.015}

        gripper_open = {'type': 'open'}

        # NOTE: important
        throw_pose_list = [throw_move,  # throw_move,
                           gripper_open, home_move, start_move, sideways_move]
        # throw_pose_list = [start_move]

        # pose_list = [start_pose, middle_pose, end_pose, start_pose]
        self.combo_move(throw_pose_list, wait=True, is_sim=is_sim)

    def throw_overhand(self, wait=True, RG2=True):
        pass

    def throw_andy(self, wait=True, is_sim=False):
        default_jacc = 8.  # 8
        default_jvel = 15.0  # 10
        toss_jacc = 25  # 25.
        toss_jvel = 3.2  # 3.2
        pretoss_jconf = np.asarray(
            [0., -45., 90., -078.9, -90., 0.])*np.pi/180.0
        # np.asarray([0., -45., 90., -098.9, -90., 0.]) * \
        # np.pi/180.0  # per email
        posttoss_jconf = np.asarray([0., -065.8, 015.1, -130.1, -90., 0.]) * \
            np.pi/180.0
        # np.asarray([0., -057.8, 035.1, -142.1, -90., 0.]) * \
        # np.pi/180.0  # per email
        pretoss_blend_radius = 0.09  # TODO: this does get used at all?
        # toss_blend_radius = 0.7 # BLEND FAIL
        toss_blend_radius = 0.6
        # toss_blend_radius = 0.05

        tcp_msg = "def process():\n"
        tcp_msg += '    socket_open("127.0.0.1",63352,"gripper_socket")\n'
        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("SPE", 255,
                                                                   self.socket_name)
        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("FOR", 0,
                                                                   self.socket_name)
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=%f)\n' % \
            (pretoss_jconf[0], pretoss_jconf[1], pretoss_jconf[2], pretoss_jconf[3],
             pretoss_jconf[4], pretoss_jconf[5], default_jacc, default_jvel, pretoss_blend_radius)
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=%f)\n' % \
            (posttoss_jconf[0], posttoss_jconf[1], posttoss_jconf[2], posttoss_jconf[3],
             posttoss_jconf[4], posttoss_jconf[5], toss_jacc, toss_jvel, toss_blend_radius)
        # tcp_msg += '    set_digital_out(8,False)\n'  # for RG2 gripper

        tcp_msg += "    socket_set_var(\"{}\",{},\"{}\")\n".format("POS", 0,
                                                                   self.socket_name)
        # tcp_msg += "    sync()\n"
        tcp_msg += '    movej([%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0.0,r=0.0)\n' % \
            (pretoss_jconf[0], pretoss_jconf[1], pretoss_jconf[2], pretoss_jconf[3],
             pretoss_jconf[4], pretoss_jconf[5], default_jacc, default_jvel)
        tcp_msg += '    socket_close("gripper_socket")\n'
        tcp_msg += 'end\n'
        self.send_program(tcp_msg, is_sim=is_sim)

        # tcp_msg += self._format_move("movej", pretoss_jconf, default_jacc,
        # default_jvel, pretoss_blend_radius, time=0, prefix="") + "\n"
        # tcp_msg += self._format_move("movej", posttoss_jconf, toss_jacc,
        # toss_jvel, toss_blend_radius, time=0, prefix="") + "\n"
        # tcp_msg += self._format_move("movej", pretoss_jconf, default_jacc,
        # default_jvel, radius=0, time=0, prefix="") + "\n"

        if wait:
            joint_flag = True
            self._wait_for_move(target=pretoss_jconf,
                                threshold=self.pose_tolerance, joints=joint_flag)
            return self.get_state('cartesian_info')

        print('done with toss')
        # self.send_program(prog, is_sim=is_sim)

    def throw(self, is_sim=False):
        self.close_gripper()
        # currently hard coded positions
        # acc, vel = 1.4, 1.05 # Safe
        # acc, vel = 8, 3 # Default
        # acc, vel = 15, 10
        # K = 8.5
        K = 2
        acc, vel = 1.4 * K, K
        start_position = [0.350, 0.000, 0.250, 2.12, -2.21, -0.009]
        start_move = {'type': 'p',
                      'pose': start_position,
                      # 'acc': None, 'vel': None, 'radius': 0.2}
                      'acc': acc, 'vel': vel, 'radius': 0.001}

        # curled_position = [0.350, 0.000, 0.250, 1.75, -1.80, -0.62]
        # curled_move = {'type': 'p',
        # 'pose': curled_position,
        # 'acc': acc, 'vel': vel, 'radius': 0.000}

        # throw_position = [0.597, 0.000, 0.640, 2.26, -2.35, 2.24]
        throw_position = [0.550, 0.000, 0.550, 2.26, -2.35, 2.24]
        # throw_position = [0.567, 0.000, 0.580, 2.38, -2.37, 1.60]
        throw_move = {'type': 'p',
                      'pose': throw_position,
                      'acc': acc, 'vel': vel, 'radius': 0.150}

        throw2_position = np.array(throw_position) + \
            np.array([0.050, 0.000, 0.105, 0, 0, 0])
        throw2_position = throw2_position.tolist()
        throw2_move = {'type': 'p',
                       'pose': throw2_position,
                       'acc': acc, 'vel': vel, 'radius': 0.01}  # max =0.05*1.4
        # return_position = [0.590, 0.000, 0.620, 2.38, -2.37, 1.60]
        # return_move = {'type': 'p',
        # 'pose': return_position,
        # 'acc': 3.5, 'vel': 3.5, 'radius': 0.100}

        home_position = np.array(start_position) + \
            np.array([0.010, 0, 0.210, 0, 0, 0])
        home_position = home_position.tolist()
        home_move = {'type': 'p',
                     'pose': home_position,
                     'acc': 1.0, 'vel': 1.0, 'radius': 0.001}

        gripper_open = {'type': 'open'}

        # throw_pose_list = [curled_move, throw_move,  # throw_move,
        throw_pose_list = [throw_move, gripper_open, throw2_move,
                           home_move, start_move]
        # throw_pose_list = [start_move]

        self.combo_move(throw_pose_list, wait=True, is_sim=is_sim)

        '''
        # Pre-compute blend radius
        # blend_radius = min(abs(bin_position[1] - position[1])/2 - 0.01, 0.2)
        # tcp_command += "movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=%f)\n" % \
        # (position[0], position[1], bin_position[2],
        # tool_orientation[0], tool_orientation[1], 0.0,
        # self.joint_acc, self.joint_vel, blend_radius)
        '''

    # Move to box grabbing position
    box_grab_position = [0.5, -0.35, -0.12]
    tcp_command = "def process():\n"
    tcp_command += " set_digital_out(8,False)\n"
    tcp_command += " set_digital_out(8,True)\n"
    tcp_command += "end\n"
    self.tcp_socket.send(str.encode(tcp_command))
    self.tcp_socket.close()

    # Move to box release position
    box_release_position = [0.5, 0.08, -0.12]
    home_position = [0.49, 0.11, 0.03]
    # Block until robot reaches home position
    state_data = self.get_state()
    tool_analog_input2 = self.parse_tcp_state_data(state_data, 'tool_data')
