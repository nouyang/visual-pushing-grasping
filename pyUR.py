"""
"""
import ursecmon  # NOTE: ursecmon in same folder
import logging
import numpy as np

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"


class PyUR(object):
    def __init__(self, host, joint_vel, joint_acc, home_joint_config=None, workspace_limits=None):

        self.is_robotiq = False
        self.joint_vel = joint_vel
        self.joint_acc = joint_acc

        # use_rt=False, use_simulation=False):
        # self.host = host
        # self.csys = None
        self.logger = logging.getLogger("urx")
        self.logger.debug("Opening secondary monitor socket")

        self.secmon = ursecmon.SecondaryMonitor(host)

        # NOTE: this is for throw practice
        if home_joint_config is None:
            home_in_deg = np.array(
                [-10.5, -50.5, 125., -71.9, -277.5, -1.4]) * 1.0
            self.home_joint_config = np.deg2rad(home_in_deg)
        else:
            self.home_joint_config = home_joint_config
        self.logger.debug("Home config: " + str(self.home_joint_config))

        # self.moveto_limits = (
        #   [[0.300, 0.600], [-0.250, 0.180], [0.195, 0.571]])

        # self.moveto_limits = np.asarray(
        # [[-0.700, -0.350], [-0.125, 0.225], [-0.290, -0.195]])  # grasp pos

        self.moveto_limits = workspace_limits
        print('in urcomm init we have limits', self.moveto_limits)
        # self.moveto_limits = np.asarray(
        # [[-0.650, -0.400], [-0.100, 0.100], [-0.300, -0.150]])  # for calib
        # Tool pose tolerance for blocking calls (meters)
        # HACK lower tolerance for now 31 July,bent wrist move not completing
        # self.pose_tolerance = [0.002, 0.002, 0.002, 0.010, 0.010, 0.010]
        self.pose_tolerance = [0.005, 0.005, 0.005, 0.020, 0.020, 0.020]

        self.socket_name = "gripper_socket"

        self.socket_open_str = '\tsocket_open("127.0.0.1", 63352, "gripper_socket")\n'
        self.socket_close_str = '\tsocket_close("gripper_socket")\n'

        """
        FOR is the variable
        range is 0 - 255
        0 is no force
        255 is full force
        """
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        """
        POS is the variable
        range is 0 - 255
        0 is open
        255 is closed
        """
        self.max_float_length = 6  # according to python-urx lib, UR may have max float length

    # -- Gripper commands

    def activate_gripper(self):
        if self.is_robotiq:
            prog = "def actGrip():\n"
            # Activate gripper
            prog += self.socket_close_str
            prog += self.socket_open_str
            # TODO: does this cause the gripper to open and close? to acitvate
            prog += "socket_set_var(\"{}\",{},\"{}\")\n".format("ACT", 1,
                                                                self.socket_name)
            prog += "socket_set_var(\"{}\",{},\"{}\")\n".format("GTO", 1,
                                                                self.socket_name)
            prog += "end\n"
        else:
            prog =  \
                '''
            def start_rg2():
                set_tool_voltage(0)
                sleep(1.0)
                set_digital_out(8, False)
                set_digital_out(9, False)
                set_tool_voltage(24)
                timeout = 0
                while get_digital_in(9) == False:
                    timeout = timeout+1
                    # sleep(0.008)
                    sleep(0.005)
                    if timeout > 800:
                        # wait at most 5 secs
                        textmsg("breaking")
                        break
                    end
                end

                count = 0
                textmsg("beginning loop")
                set_digital_out(9, False)
                while True:
                    textmsg(count)
                    set_digital_out(8, True)
                    sleep(1)
                    set_digital_out(8, False)
                    sleep(1)
                    count = count + 1
                end
            end
            '''
        self.logger.debug("Activating gripper")
        # self.send_program(prog)

    # We also talk to Robotiq 2F-85 gripper through the UR5 "API"

    def open_gripper(self, async=False, robotiq=False):
        if self.is_robotiq:
            prog = "def openGrip():\n"
            prog += self.socket_close_str
            prog += self.socket_open_str
            prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("SPE", 255,
                                                                  self.socket_name)
            prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("POS", 0,
                                                                  self.socket_name)
            prog += "end\n"
            self.send_program(prog)
        else:  # RG2
            self.send_program("set_digital_out(8,False)\n")
        self.logger.debug("opening gripper")

    def close_gripper(self, async=False):
        if self.is_robotiq:
            prog = "def closeGrip():\n"
            prog += self.socket_close_str
            prog += self.socket_open_str
            prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("FOR", 20,
                                                                  self.socket_name)
            prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("SPE", 255,
                                                                  self.socket_name)
            prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("POS", 255,
                                                                  self.socket_name)
            prog += "end\n"
        else:  # rg2
            self.send_program("set_digital_out(8,True)\n")
        self.logger.debug("Closing gripper")

        gripper_fully_closed = self.check_grasp()
        # gripper_fully_closed = True
        return gripper_fully_closed

    def check_grasp(self):
        if self.is_robotiq:
            prog = "def setAnalogOutToGripPos():\n"
            prog += self.socket_close_str
            prog += self.socket_open_str
            prog += '\trq_pos = socket_get_var("POS","gripper_socket")\n'
            prog += "\tset_standard_analog_out(0, rq_pos / 255)\n"
            prog += "end\n"
            self.send_program(prog)
            # TODO: read analog out 0, determine values to compare to
            # tool_pos = self.get_state('tool_data')
            # return tool_pos > 9  # TODO
        else:
            # Gripper did not close all the way
            return self.get_state('gripper_width') > 0.26
            # tool_analog_input2 > 0.26

            # -- Data commands

    def get_state(self, subpackage):
        def get_joint_data(_log=True):
            jts = self.secmon.get_joint_data()
            joint_positions = [jts["q_actual0"], jts["q_actual1"],
                               jts["q_actual2"], jts["q_actual3"],
                               jts["q_actual4"], jts["q_actual5"]]
            if _log:
                self.logger.debug("Received joint data from robot: %s",
                                  str(joint_positions))
            return joint_positions

        def get_cartesian_info(_log=True):
            pose = self.secmon.get_cartesian_info()
            if pose:
                pose = [pose["X"], pose["Y"], pose["Z"],
                        pose["Rx"], pose["Ry"], pose["Rz"]]
            if _log:
                self.logger.debug(
                    "Received pose data from robot: %s", str(pose))
            return pose

        def get_gripper_width(_log=True):
            # TODO: is this a value b/tw 0 and 10?
            if self.is_robotiq:
                width = self.secmon.get_all_data()["ToolData"]["analogInput2"]
            else:
                width = self.secmon.get_tool_analog_in(2)
            if _log:
                self.logger.debug(
                    "Received gripper(is robotiq? % s) width from robot: % s" %
                    (str(self.is_robotiq), str(width)))
            return width

        parse_functions = {'joint_data': get_joint_data,
                           'cartesian_info': get_cartesian_info, 'gripper_width':
                           get_gripper_width}
        return parse_functions[subpackage]()

    def send_program(self, prog, is_sim=False):
        # mostly adding a printout for ease of debugging
        if not is_sim:
            self.logger.info("Sending program: " + prog)
            self.secmon.send_program(prog)
        else:
            self.logger.info("SIM. Would have sent program: " + prog)

    # -- Utils

    def _btw(self, a, min, max):
        if (a >= min) and (a <= max):
            return True
        return False

    def _is_safe(self, position, limits):
        safe = self._btw(position[0], limits[0][0], limits[0][1]) and \
            self._btw(position[1], limits[1][0], limits[1][1]) and \
            self._btw(position[2], limits[2][0], limits[2][1])
        return safe

    def _format_move(self, command, tpose, acc, vel, radius=0, time=0, prefix=""):
        # prefix= p for position, none for joints
        tpose = [round(i, self.max_float_length) for i in tpose]
        # can i specifiy time?
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        tpose.append(time)
        return "\t{}({}[{}, {}, {}, {}, {}, {}], a={}, v={}, r={}, t={})\n".format(command, prefix, *tpose)

   # tcp_command += " set_digital_out(8,False)\n"
   # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.09)\n" \
   # % (position[0], position[1], position[2] + 0.1, tool_orientation[0],
   # tool_orientation[1], 0.0, self.joint_acc * 0.5, self.joint_vel * 0.5)
   # tcp_command += " movej(p[%f,%f,%f,%f,%f,%f],a=%f,v=%f,t=0,r=0.00)\n" \
   # % (position[0], position[1], position[2], tool_orientation[0],
   # tool_orientation[1], 0.0, self.joint_acc * 0.1, self.joint_vel * 0.1)
   # tcp_command += " set_digital_out(8,True)\n"

    # -- Move commands

    def move_to(self, position, orientation, vel=None, acc=None, radius=0,
                wait=True, override_safety=False):
        if vel is None:
            vel = self.joint_vel
        if acc is None:
            acc = self.joint_acc
        # position ins meters, orientation is axis-angle

        # TODO: Remove this hardcoding
       # moveto_lims = np.asarray(
       #     [[-0.700, -0.350], [-0.125, 0.225], [-0.300, -0.000]])  # grasp pos

        if self._is_safe(position, self.moveto_limits) or override_safety:
            prog = "def moveTo():\n"
            # t = 0, r = radius
            if orientation is None:
                self.logger.info(
                    "Attempting to move position but keep orientation")
                print('keep current orient')
                orientation = self.get_state('cartesian_info')[3:]
            else:
                print('new orientation', orientation)

            prog += self._format_move("movel", np.concatenate((position, orientation)),
                                      acc=acc, vel=vel, prefix="p")
            prog += "end\n"
            self.send_program(prog)
        else:
            print("NOT Safe. NOT moving to: %s, due to LIMITS: %s",
                  position, self.moveto_limits)
            # self.logger.info("NOT Safe. NOT moving to: %s, due to LIMITS: %s",
            # position, self.moveto_limits)
        if wait:
            print('waiting for', position, orientation)
            self._wait_for_move(np.concatenate((position, orientation)),
                                joints=False)
            print('done waiting')

    def move_joints(self, joint_configuration, vel=None, acc=None, wait=True):
        if vel is None:
            vel = self.joint_vel
        if acc is None:
            acc = self.joint_acc

        # specified in radians
        prog = "def moveJoint():\n"
        prog += self._format_move("movel", joint_configuration,
                                  vel=vel, acc=acc, prefix="")
        prog += "end\n"
        self.send_program(prog)
        if wait:
            self._wait_for_move(joint_configuration, joints=True)

    def go_home(self):
        self.logger.debug("Going home.")
        self.move_joints(self.home_joint_config)

    # def get_gripper_width(self):
        # return self.get_state('tool_data')

    def combo_move(self, moves_list, wait=True, is_sim=False):
        """
        Example use:
        pose_list = [ {type:p, vel:0.1, acc:0.1, radius:0.2},
                    {type: open}]
        """
        prog = "def combo_move():\n"
        # prog += self.socket_close_str
        prog += self.socket_open_str

        for idx, a_move in enumerate(moves_list):

            if a_move["type"] == 'open':
                if self.is_robotiq:
                    prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("SPE", 255,
                                                                          self.socket_name)
                    prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("POS", 0,
                                                                          self.socket_name)
                else:
                    prog += "\tset_digital_out(8, False)"

            elif a_move["type"] == 'close':
                if self.is_robotiq:
                    prog += "\tsocket_set_var(\"{}\",{},\"{}\")\n".format("POS",
                                                                          255,
                                                                          self.socket_name)
                else:
                    prog += "\tset_digital_out(8, True)"

            else:
                if 'radius' not in a_move:
                    a_move['radius'] = 0.01
                if 'acc' not in a_move:
                    # acc = self.joint_acc
                    a_move['acc'] = self.joint_acc
                if 'vel' not in a_move:
                    a_move['vel'] = self.joint_vel
                if idx == (len(moves_list) - 1):
                    radius = 0.001
                    # acc = self.joint_acc
                    # vel = self.joint_vel
                acc, vel, radius = a_move["acc"], a_move["vel"], a_move["radius"]

                # WARNING: this does not have safety checks!
                if a_move["type"] == 'j':
                    prog += self._format_move(
                        "movel", a_move['pose'], acc, vel, radius, prefix="") + "\n"
                elif a_move["type"] == 'p':
                    prog += self._format_move(
                        'movel', a_move['pose'], acc, vel, radius, prefix="p") + "\n"
        prog += "end\n"

        self.send_program(prog, is_sim=is_sim)

        if wait:
            joint_flag = False
            if moves_list[-1]['type'] == 'j':
                joint_flag = True
            self._wait_for_move(target=moves_list[-1]['pose'],
                                threshold=self.pose_tolerance, joints=joint_flag)
            return self.get_state('cartesian_info')

    def is_running(self):
        """
        Return True if robot is running (not
        necessary running a program, it might be idle)
        """
        return self.secmon.running

    def _wait_for_move(self, target, threshold=None, joints=False):
        """
        Wait for a move to complete. Unfortunately there is no good way to know
        when a move has finished so for every received data from robot we
        compute a dist equivalent and when it is lower than 'threshold' we
        return.
        if threshold is not reached within timeout, an exception is raised
        """
        self.logger.debug(
            "Waiting for move completion using threshold %s and target %s", threshold, target)
        if threshold is None:
            # threshold = [0.001] * 6
            threshold = self.pose_tolerance
            self.logger.debug("No threshold set, setting it to %s", threshold)
        while True:
            if not self.is_running():
                # raise RobotException("Robot stopped")
                self.logger.exception("ROBOT STOPPED!")
            if joints:
                actual_pose = self.get_state('joint_data')
            else:
                actual_pose = self.get_state('cartesian_info')

            dist = [np.abs(actual_pose[j] - target[j]) for j in range(6)]
            self.logger.debug(
                "distance to target is: %s, target dist is %s", dist, threshold)
            # if all([np.abs(actual_pose[j] - target[j]) < self.pose_tolerance[j] for j in range(6)]):
            #  TODO: we are having some issue where the rx ry rz over ethernet !=
            # pendant rx ry rz
            if all([np.abs(actual_pose[j] - target[j]) < self.pose_tolerance[j] for j in range(3)]):
                self.logger.debug(
                    "We are threshold(%s) close to target, move has ended", threshold)
                return

    '''
    def _wait_for_move(self, target, threshold=None, timeout=5, joints=False):
        """
        wait for a move to complete. Unfortunately there is no good way to know when a move has finished
        so for every received data from robot we compute a dist equivalent and when it is lower than
        'threshold' we return.
        if threshold is not reached within timeout, an exception is raised
        """
        self.logger.debug(
            "Waiting for move completion using threshold %s and target %s", threshold, target)
        start_dist = self._get_dist(target, joints)
        if threshold is None:
            threshold = start_dist * 0.8
            if threshold < 0.001:  # roboten precision is limited
                threshold = 0.001
            self.logger.debug("No threshold set, setting it to %s", threshold)
        count = 0
        while True:
            if not self.is_running():
                raise RobotException("Robot stopped")
            dist = self._get_dist(target, joints)
            self.logger.debug(
                "distance to target is: %s, target dist is %s", dist, threshold)
            if not self.secmon.is_program_running():
                if dist < threshold:
                    self.logger.debug(
                        "we are threshold(%s) close to target, move has ended", threshold)
                    return
                count += 1
                if count > timeout * 10:
                    raise RobotException("Goal not reached but no program has been running for {} seconds. dist is {}, threshold is {}, target is {}, current pose is {}".format(
                        timeout, dist, threshold, target, URRobot.getl(self)))
            else:
                count = 0
    '''

    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist ** 0.5

    def _get_joints_dist(self, target):
        joints = self.getj(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
