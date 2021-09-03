import numpy as np
import cv2

from pyrep.objects.vision_sensor import VisionSensor

from .camera import Camera
from ._util import euler2rotm
from robot_tool.utils.logger import warn, error


def VirtualEye(backend="V-REP", **kwargs):
    if backend == "bullet":
        return _BulletEye()
    elif backend == "V-REP":
        return _VREPEye(**kwargs)
    else:
        error("Virtual Camera backbend must be one of {bullet, V-REP}!")
        raise NotImplementedError


class _BulletEye(Camera):
    def __init__(self):
        pass

    def get_data(self):
        pass

    def get_camera_params(self):
        return {
            "backend": "bullet",
        }

    def close(self):
        pass


class _VREPEye(Camera):
    def __init__(self, name):
        self._cam_name = name
        self.camera = VisionSensor(name)

        cam_pos, cam_orn = self.camera.get_position(), self.camera.get_orientation()
        cam_trans = np.eye(4, 4)
        cam_trans[0:3, 3] = np.asarray(cam_pos)
        cam_orientation = [-cam_orn[0], -cam_orn[1], -cam_orn[2]]
        cam_rotm = np.eye(4, 4)
        cam_rotm[0:3, 0:3] = np.linalg.inv(euler2rotm(cam_orientation))
        self.cam_pose = np.dot(cam_trans, cam_rotm)
        if "persp" in self._cam_name:
            self.cam_intri = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
        else:
            self.cam_intri = None
        self.depth_scale = 1

    def get_data(self):
        rgb_img = self.camera.capture_rgb()
        # Based on https://github.com/andyzeng/visual-pushing-grasping/blob/master/utils.py
        rgb_img *= 255
        rgb_img = rgb_img.astype(np.uint8)
        rgb_img = np.flip(rgb_img, 0).copy()  # Image is upside-down
        rgb_img = np.fliplr(rgb_img).copy()
        depth_img = self.camera.capture_depth(in_meters=True)
        depth_img = np.flip(depth_img, 0).copy()  # Image is upside-down
        depth_img = np.fliplr(depth_img).copy()

        return rgb_img, depth_img

    def get_camera_params(self):
        return {
            "image_type": "bgr",
            "cam_intri": self.cam_intri.copy(),
            "cam_pose": self.cam_pose.copy(),
            "cam_name": self._cam_name,
            "backend": "V-REP",
        }

    def close(self):
        pass