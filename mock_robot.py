import random
import numpy as np

class MockRobot(object):

    def __init__(self):
        self.cam_intrinsics = 1.0
        self.cam_depth_scale = 1.0
        self.cam_pose = 1.0
        self.use_cam = True

        try:
            # -- Is the camera connected? if so, use it
            # -- Use when want to test camera but not move the robot 
            from real.camera import Camera
            self.camera = Camera()
            self.cam_intrinsics = self.camera.intrinsics
            # Load camera pose (from running calibrate.py), intrinsics and depth scale
            self.cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
            self.cam_depth_scale = np.loadtxt(
                "real/camera_depth_scale.txt", delimiter=" ")
        except:
            self.use_cam = False
            print('Failed to open camera)

    # Get image
    def get_camera_data(self):
        if self.use_cam:
            image, depth = camera.get_data()
        else:
            image = np.random.uniform(0, 1, size=(112, 112, 3)).astype(np.float32)
            depth = np.random.uniform(0, 1, size=(112, 112)).astype(np.float32)
        return image, depth

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        return random.choice([True, False])

    def throw(self, position, heightmap_rotation_angle, workspace_limits):
        pass

    def restart_real(self):
        pass
