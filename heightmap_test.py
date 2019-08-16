import numpy as np
import cv2
from logger import Logger
import robot
import utils
from real.camera import Camera
import os

debug_dir = './heightmap_debug'
# logger = Logger(False, debug_dir)

# color_img, depth_img = robot.get_camera_data()
# Apply depth scale from calibration
# depth_img = depth_img * robot.cam_depth_scale

# color_img, depth_img = robot.get_camera_data()
camera = Camera()
# cam_intrinsics = self.camera.intrinsics

cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
cam_depth_scale = np.loadtxt(
    "real/camera_depth_scale.txt", delimiter=" ")
cam_intrinsics = camera.intrinsics
print(cam_depth_scale, cam_intrinsics)

workspace_limits = np.asarray(
    [[-0.710, -0.360], [-0.115, 0.235], [-0.400, -0.310]])  # with gripper horizontal
# [[-0.700, -0.350], [-0.125, 0.225], [-0.420, -0.300]])  # with gripper vertical

# Cam intrinsices are sent by camera directly
# cam_intrinsics = np.array([[920.723,   0., 653.2795],
# [0., 920.02124, 362.55386],
# [0.,   0.,   1.]])

# color_img = cv2.imread('color.png', 0)
# depth_img = cv2.imread('depth.png', 0)
color_img, depth_img = camera.get_data()
print('color image shape', color_img.shape)
depth_img = depth_img * cam_depth_scale
k = 1.
# Scale image, to change heightmap resolution, so resultant image is 224x224
color_img = cv2.resize(color_img, (0, 0), fx=k, fy=k)
depth_img = cv2.resize(depth_img, (0, 0), fx=k, fy=k)
heightmap_resolution = 0.001*(1./k)
# depth_img = depth_img.astype(float) * depth_scale

color_heightmap, depth_heightmap = utils.get_heightmap(
    color_img, depth_img, cam_intrinsics, cam_pose, workspace_limits, heightmap_resolution)

valid_depth_heightmap = depth_heightmap.copy()
valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0


# logger.save_heightmaps(1, color_heightmap, valid_depth_heightmap, '0')

color_image = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
print('color image saved shape', color_image.shape)
cv2.imwrite(os.path.join(debug_dir, 'color.png'), color_image)
# Save depth in 1e-4 meters
depth_image = np.round(depth_img * 10000).astype(np.uint16)
cv2.imwrite(os.path.join(debug_dir, 'depth.png'), depth_image)

color_heightmap = cv2.cvtColor(color_heightmap, cv2.COLOR_RGB2BGR)
print('color heighmap saved shape', color_heightmap.shape)
cv2.imwrite(os.path.join(debug_dir, 'color_heightmap.png'), color_heightmap)

# Save depth in 1e-5 meters
depth_heightmap = np.round(depth_heightmap * 100000).astype(np.uint16)
cv2.imwrite(os.path.join(debug_dir, 'depth_heightmap.png'), depth_heightmap)
