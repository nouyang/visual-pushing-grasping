import random
import numpy as np


class MockRobot(object):

    def __init__(self):

        self.cam_intrinsics = 1.0
        self.cam_depth_scale = 1.0
        self.cam_pose = 1.0

    def get_camera_data(self):

        image = np.random.uniform(0, 1, size=(112, 112, 3)).astype(np.float32)
        depth = np.random.uniform(0, 1, size=(112, 112)).astype(np.float32)

        return image, depth

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):

        return random.choice([True, False])

    def throw(self, position, heightmap_rotation_angle, workspace_limits):

        pass

    def throw_andy(self, wait=True, is_sim=False):

        pass


def restart_real(self):

    pass
