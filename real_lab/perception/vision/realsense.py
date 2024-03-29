import pyrealsense2 as rs

import numpy as np
from collections import Iterable
from scipy.spatial.transform import Rotation as R

from .camera import Camera

DEFAULT_REALSENSE_CAMERA_ROTATION = np.array([-0.874125361082, -0.0270816605365, -0.00440328852424, 0.484924785742])
DEFAULT_REALSENSE_CAMERA_TRANSLATION = np.array([[-0.0130321939147], [-1.22888842636], [0.479256365879]])


class RealSense(Camera):
    def __init__(self, cam_rotation=None, cam_translation=None, init_params=None, post_process=None, model=None):
        if cam_rotation is None:
            cam_rotation = DEFAULT_REALSENSE_CAMERA_ROTATION
        if cam_translation is None:
            cam_translation = DEFAULT_REALSENSE_CAMERA_TRANSLATION

        self.pipeline = rs.pipeline()

        if init_params is None:
            init_params = {}
            config = rs.config()
            if model == "SR300":
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
            else:
                config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            init_params['config'] = config

            # align depth to color
            init_params['align'] = rs.stream.color

        self.init_params = init_params
        self.aligned_frames = rs.align(init_params['align'])
        config_params = self.init_params['config']
        profile = self.pipeline.start(config_params)

        # color_sensor = profile.get_device().first_color_sensor()
        depth_sensor = profile.get_device().first_depth_sensor()
        # For different situation, set different depth_units,
        # since different depth_units lead to different valid depth range.
        # like, for depth_units is 1e-6, range is 0.5 ~ 0.62 meters.
        if model == "SR300":
            # depth_sensor.set_option(rs.option.depth_units, 0.000125)
            pass
        else:
            depth_sensor.set_option(rs.option.depth_units, 0.0001)

        self.depth_scale = depth_sensor.get_depth_scale()
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.aligned_frames.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        while not depth_frame or not color_frame:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.aligned_frames.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

        color_cam_intri = color_frame.get_profile().as_video_stream_profile().get_intrinsics()

        # depth_cam_intri = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()  # for ptc
        r = R.from_quat(cam_rotation)
        cam_rotation = r.as_matrix()
        self.cam_pose = np.zeros((4, 4), dtype=np.float32)
        self.cam_pose[:3, :3] = cam_rotation
        self.cam_pose[:3, 3:] = cam_translation
        self.cam_pose[-1, -1] = 1

        self.cam_intri = np.zeros([3, 3])
        # self.depth_cam_intri = np.zeros([3, 3])

        self.cam_intri[0][0] = color_cam_intri.fx
        self.cam_intri[1][1] = color_cam_intri.fy
        self.cam_intri[0][2] = color_cam_intri.ppx
        self.cam_intri[1][2] = color_cam_intri.ppy
        self.cam_intri[2][2] = 1

        self._cam_name = "RealSense"
        self.post_process = post_process

    def get_data(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.aligned_frames.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        while not depth_frame or not color_frame:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.aligned_frames.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

        depth_frame = self._post_process(depth_frame)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data()) * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())

        return color_image, depth_image

    def _post_process(self, frame):
        assert isinstance(frame, rs.frame)
        if isinstance(self.post_process, Iterable):
            tmp_frame = frame.copy()
            for f in self.post_process:
                assert isinstance(f, rs.filter), "Each element in post_process should be 'rs.filter'"
                tmp_frame = f.process(tmp_frame)
            return tmp_frame
        elif isinstance(self.post_process, rs.filter):
            return self.post_process.process(frame)
        elif self.post_process is None:
            # Do nothing
            return frame
        else:
            raise TypeError("post_process should be one of '[rs.filter, ...]', 'rs.filter', or None")

    def close(self):
        self.pipeline.stop()

    def get_camera_params(self):
        return {
            "image_type": "bgr",
            "init_params": self.init_params,
            "depth_scale": self.depth_scale,
            "cam_intri": self.cam_intri.copy(),
            "cam_pose": self.cam_pose.copy(),
            "cam_name": self._cam_name
        }

    def auto_calibration(self):
        pass
