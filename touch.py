#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from real.camera import Camera
from robot import Robot


# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = "10.75.15.199"  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = "10.75.15.199"  # IP and port to robot arm as TCP client (UR5)
rtc_port = 30001
# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
# tool_orientation = [-1.22, 1.19, -1.17]  # gripper facing upward, for calib
# tool_orientation = [2.339, 2.396, -2.480]
# tool_orientation = [2.24, 2.30, -0.00]
tool_orientation = [2.24, 2.30, -0.00]

workspace_limits = np.asarray(
    # [[0.400, 0.600], [-0.250, 0.150], [0.195, 0.460]])
    # [[-0.600, -0.400], [-0.190, 0.120], [-0.460, -0.200]])  # gripper is fat
    [[-0.750, -0.550], [-0.250, 0.150], [-0.305, -0.090]])

# home_rad = np.deg2rad([16.6, -26.5, 116.8, -184.6, -90.4, 181.4])
home_rad = np.deg2rad([0, -13.7, 90, -165., -90., -5.])

# Move robot to home pose
is_sim = False
obj_mesh_dir = None
num_obj = None


is_testing = False
test_preset_cases = None
test_preset_file = None

robot = Robot(is_sim, obj_mesh_dir, num_obj, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              is_testing, test_preset_cases, test_preset_file,
              home_joint_config=home_rad)

print('Robot in touch.py has gone home')
robot.r.open_gripper()

# Slow down robot
# robot.joint_acc = 1.4
# robot.joint_vel = 1.05

# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = robot.get_camera_data()


def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Button Down detected at ', x, y)
        global camera, robot, click_point_pix
        click_point_pix = (x, y)

        # Get click point in camera coordinates
        click_z = camera_depth_img[y][x] * robot.cam_depth_scale
        click_x = np.multiply(
            x-robot.cam_intrinsics[0][2], click_z/robot.cam_intrinsics[0][0])
        click_y = np.multiply(
            y-robot.cam_intrinsics[1][2], click_z/robot.cam_intrinsics[1][1])
        if click_z == 0:
            print('click_z was zero, doing nothing')
            return
        click_point = np.asarray([click_x, click_y, click_z])
        click_point.shape = (3, 1)

        # Convert camera to robot coordinates
        # camera2robot = np.linalg.inv(robot.cam_pose)
        camera2robot = robot.cam_pose
        target_position = np.dot(
            camera2robot[0:3, 0:3], click_point) + camera2robot[0:3, 3:]

        target_position = target_position[0:3, 0]
        print('Moving to ', target_position, ' with z offset of 0.200')
        target_position[2] += 0.200
        robot.r.move_to(target_position, tool_orientation,
                        override_safety=True)


# Show color and depth frames
cv2.namedWindow('color')
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    camera_color_img, camera_depth_img = robot.get_camera_data()
    bgr_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        bgr_data = cv2.circle(bgr_data, click_point_pix, 7, (0, 0, 255), 2)
    cv2.imshow('color', bgr_data)
    cv2.imshow('depth', camera_depth_img)

    if cv2.waitKey(1) == ord('c'):
        break

cv2.destroyAllWindows()
