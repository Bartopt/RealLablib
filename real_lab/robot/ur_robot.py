import numpy as np

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from scipy.spatial.transform.rotation import Rotation as R

from ..end_effector.robotiq_gripper_control import RobotiqGripper
from ..end_effector.robotiq_usb_ctrl import RobotiqUSBCtrlGripper
from .robot import Robot


class URRobot(Robot):
    def __init__(self, robot_ip, rtu_gripper=False, home_joint_position=None):
        self.robot_ip = robot_ip
        self.rtu_gripper = rtu_gripper

        self.rtde_c = RTDEControl(self.robot_ip)
        self.rtde_r = RTDEReceive(self.robot_ip)

        if rtu_gripper:
            self.gripper = RobotiqUSBCtrlGripper('/dev/ttyUSB0')
            self.gripper.activate()
            self.gripper.send_commond()
            self.gripper.reset()
            self.gripper.send_commond()
        else:
            self.gripper = RobotiqGripper(self.rtde_c)
            self.gripper.set_speed(100)
            self.gripper.activate()

        self._default_joint_vel = 1.05
        self._default_joint_acc = 1.4
        self._default_joint_tolerance = 0.01

        self._default_tcp_vel = 0.3
        self._default_tcp_acc = 1.2
        self._default_tcp_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        if home_joint_position is None:
            self._home_joint_position = [88.41 * np.pi / 180, -128.30 * np.pi / 180, 126.03 * np.pi / 180,
                                         -87.80 * np.pi / 180, -89.46 * np.pi / 180, 138 * np.pi / 180]
        else:
            self._home_joint_position = home_joint_position

        self.reset()

    def get_observation(self):
        current_pose = self.rtde_r.getActualTCPPose()
        return current_pose[:3], current_pose[3:]

    def get_robot_params(self):
        return {
            "robot_ip": self.robot_ip,
            "is_rtu_gripper": self.rtu_gripper,
            "current_pose": self.rtde_r.getActualTCPPose()
        }

    def reset(self):
        self.move_joints(self._home_joint_position)
        self.gripper.open()

    def apply_action(self, *motor_commands, acc=None, vel=None):
        x, y, z, rx, ry, rz = motor_commands

        ori = [rx, ry, rz]
        r = R.from_euler('xyz', ori, degrees=False)
        orn = list(r.as_rotvec())
        target_pos = [x, y, z]

        if acc is None:
            acc = self._default_tcp_acc
        if vel is None:
            vel = self._default_joint_vel
        self.rtde_c.moveL(pose=target_pos + orn, acceleration=acc, speed=vel)

    def move_joints(self, joint_configuration, acc=None, vel=None):
        if acc is None:
            acc = self._default_joint_acc
        if vel is None:
            vel = self._default_joint_vel

        self.rtde_c.moveJ(q=joint_configuration, acceleration=acc, speed=vel)

    def close(self):
        self.rtde_c.stopScript()

    def go_home(self):
        self.move_joints(self._home_joint_position)