import os
import time

import numpy as np
from pybullet_robots import BulletRobot

import logging
from .panda_robot_config import ROBOT_CONFIG


class PandaArm(BulletRobot):
    """
    Bullet simulation interface for the Franka Panda Emika robot

    Available methods (for usage, see documentation at function definition):
        - exec_position_cmd
        - exec_position_cmd_delta
        - move_to_joint_position
        - move_to_joint_pos_delta
        - exec_velocity_cmd
        - exec_torque_cmd
        - inverse_kinematics
        - untuck
        - tuck
        - q_mean
        - state
        - angles
        - n_joints
        - joint_limits
        - joint_names

        - jacobian*
        - joint_velocities*
        - joint_efforts*
        - ee_pose*
        - ee_velocity*
        - inertia*
        - inverse_kinematics*
        - joint_ids*
        - get_link_pose*
        - get_link_velocity*
        - get_joint_state*
        - set_joint_angles*
        - get_movable_joints*
        - get_all_joints*
        - get_joint_by_name*
        - set_default_pos_ori*
        - set_pos_ori*
        - set_ctrl_mode*

        *These methods can be accessed using the self._bullet_robot object from this class.
         Documentation for these methods in BulletRobot class. Refer bullet_robot.py
    """

    def __init__(self, robot_description, uid, enforce_joint_limits=False, config=None):
        """
        Constructor of the Panda robot class.

        :param robot_urdf: robot description file (urdf, .bullet, etc.)
        :param enforce_joint_limits: Enforce joint limits or not
        :param uid: server id of PyBullet
        :param config: optional config file for specifying robot information

        :type robot_urdf: str
        :type enforce_joint_limits: bool
        :type uid: int
        :type config: dict
        """
        self._ready = False

        BulletRobot.__init__(self, robot_description, uid=uid, enforce_joint_limits=enforce_joint_limits, config=config)

        # by default, set FT sensor at last fixed joint
        self._ft_joints = [self._all_joints[-1]]
        self.set_ft_sensor_at(self._ft_joints[0])

        self._tuck = [-0.017792060227770554, -0.7601235411041661, 0.019782607023391807, -2.342050140544315,
                      0.029840531355804868, 1.5411935298621688, 0.7534486589746342]
        self._untuck = self._tuck
        self.move_to_joint_position(self._tuck)

        self._ready = True

    def exec_position_cmd(self, cmd):
        """
        Execute position command. Use for position controlling.

        :param cmd: joint position values
        :type cmd: [float] len: self._nu

        """
        self.set_joint_positions_cmd(cmd, self.get_movable_joints())

    def exec_position_cmd_delta(self, cmd):
        """
        Execute position command by specifying difference from current positions. Use for position controlling.

        :param cmd: joint position delta values
        :type cmd: [float] len: self._nu

        """
        self.set_joint_positions_cmd(self.angles() + cmd, self.get_movable_joints())

    def move_to_joint_position(self, cmd):
        """
        Same as exec_position_cmd. (Left here for maintaining structure of PandaArm class from panda_robot package)

        :param cmd: joint position values
        :type cmd: [float] len: self._nu

        """
        self.reset_joint_positions(cmd)

    def move_to_joint_pos_delta(self, cmd):
        """
        Same as exec_position_cmd_delta. (Left here for maintaining structure of PandaArm class from panda_robot package)

        :param cmd: joint position delta values
        :type cmd: [float] len: self._nu

        """
        self.exec_position_cmd_delta(cmd)

    def exec_velocity_cmd(self, cmd):
        """
        Execute velocity command. Use for velocity controlling.

        :param cmd: joint velocity values
        :type cmd: [float] len: self._nu

        """
        self.set_joint_velocities_cmd(cmd, self.get_movable_joints())

    def exec_torque_cmd(self, cmd):
        """
        Execute torque command. Use for torque controlling.

        :param cmd: joint torque values
        :type cmd: [float] len: self._nu

        """
        self.set_joint_torques_cmd(cmd, self.get_movable_joints())

    # def position_ik(self, position, orientation=None):
    #     """
    #     :return: Joint positions for given end-effector pose obtained using bullet IK.
    #     :rtype: np.ndarray
    #
    #     :param position: target end-effector position (X,Y,Z) in world frame
    #     :param orientation: target end-effector orientation in quaternion format (w, x, y , z) in world frame
    #
    #     :type position: [float] * 3
    #     :type orientation: [float] * 4
    #
    #     """
    #     return self.inverse_kinematics(position, orientation)[0]

    def set_sampling_rate(self, sampling_rate=100):
        """
        (Does Nothing. Left here for maintaining structure of PandaArm class from panda_robot package)
        """
        pass

    def untuck(self):
        """
        Send robot to tuck position.
        """
        self.exec_position_cmd(self._untuck)

    def tuck(self):
        """
        Send robot to tuck position.
        """
        self.exec_position_cmd(self._tuck)

    @staticmethod
    def load_robot_models():
        """
        Add the robot's URDF models to discoverable path for robot.
        """
        import os
        BulletRobot.add_to_models_path(os.path.dirname(
            os.path.abspath(__file__)) + "/models")
