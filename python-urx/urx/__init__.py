"""
Python library to control an UR robot through its TCP/IP interface
"""
from urxOld.urrobot import RobotException, URRobot  # noqa

__version__ = "0.9.0"

try:
    from urxOld.robot import Robot
except ImportError as ex:
    print("Exception while importing math3d base robot, disabling use of matrices", ex)
    Robot = URRobot
