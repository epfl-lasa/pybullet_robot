import pybullet as pb
import numpy as np


class BulletRobotDescription(object):
    """
    Bullet Robot Description for a robotic manipulator (tested only with the Franka Panda).
    This is a collection of methods that gather and store static information from the robot description urdf
    such that this information only has to be requested from the pybullet physics engine once.

    Available methods (for usage, see documentation at function definition):
        - get_nb_joints
        - get_all_joints
        - get_joint_names
        - get_link_names
        - get_joint_dict
        - get_joint_index_by_name
        - get_link_dict
        - get_link_index_by_name
        - get_movable_joints
        - get_nb_movable_joints
        - get_ft_sensor_joints
        - get_joint_position_limits
        - get_joint_velocity_limits
        - get_joint_effort_limits
        - get_joint_limits
        - set_ft_sensor_at
        - set_default_joint_positions
        - get_default_joint_positions
        - get_base_pos_ori
    """

    def __init__(self, robot_urdf, uid, config=None):
        """
        Constructor of the Robot Description class. Gathers all information from the robot urdf description
        regarding joints and links and implements simple getter functions of robot information.

        :param robot_urdf: robot description file (urdf, .bullet, etc.)
        :param uid: server id of PyBullet
        :param config: optional config file for specifying robot information

        :type robot_urdf: str
        :type uid: int
        :type config: dict
        """
        self._uid = uid

        extension = robot_urdf.split('.')[-1]
        if extension == "urdf":
            robot_id = pb.loadURDF(robot_urdf, useFixedBase=True, physicsClientId=self._uid)
        elif extension == 'sdf':
            robot_id = pb.loadSDF(robot_urdf, useFixedBase=True, physicsClientId=self._uid)
        elif extension == 'bullet':
            robot_id = pb.loadBullet(robot_urdf, useFixedBase=True, physicsClientId=self._uid)
        else:
            robot_id = pb.loadMJCF(robot_urdf, useFixedBase=True, physicsClientId=self._uid)

        self._id = robot_id

        self._nb_joints = pb.getNumJoints(self._id, physicsClientId=self._uid)
        self._all_joints = range(self._nb_joints)
        self._all_joint_info = self._get_joint_info()
        self._all_joint_names = [joint['jointName'].decode("utf-8") for joint in self._all_joint_info]
        self._all_joint_dict = dict(zip(self._all_joint_names, self._all_joints))

        self._all_links = self._all_joints
        self._all_link_names = [joint['linkName'].decode("utf-8") for joint in self._all_joint_info]
        self._all_link_dict = dict(zip(self._all_link_names, self._all_links))

        self._movable_joints = self._find_movable_joints()
        self._nb_movable_joints = len(self._movable_joints)

        if config is not None:
            if config['ee_link_name'] is not None and config['ee_link_idx'] is not None:
                self._ee_link_name = config['ee_link_name']
                self._ee_link_idx = config['ee_link_idx']
        else:
            self._ee_link_idx, self._ee_link_name = self._use_last_defined_link()

        self._joint_position_limits = self._find_joint_position_limits()
        self._joint_velocity_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[11] for joint in
                                       self._movable_joints]
        self._joint_effort_limits = [pb.getJointInfo(self._id, joint, physicsClientId=self._uid)[10] for joint in
                                     self._movable_joints]

        self._joint_limits = [{'pos_lower': x[0], 'pos_upper': x[1], 'velocity': x[2], 'effort': x[3]}
                              for x in zip(self._joint_position_limits['lower'],
                                           self._joint_position_limits['upper'],
                                           self._joint_velocity_limits, self._joint_effort_limits)]

        self._base_pose = {}
        if config is not None:
            if config['base_position'] is not None and config['base_orientation'] is not None:
                self._base_pose['position'] = config['base_position']
                self._base_pose['orientation'] = config['base_orientation']
        else:
            self._base_pose = {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]}
        self._set_base_pose()

    def get_nb_joints(self):
        """
        Get number of all joints in the robot description.

        :return: Number of joints in the robot description
        :rtype: int
        """
        return self._nb_joints

    def get_all_joints(self):
        """
        Get indices of all joints in the robot description.

        :return: Indices of all joints in the robot description
        :rtype: list of int
        """
        return self._all_joints

    def get_joint_names(self):
        """
        Get list with the names of all joints in the robot description.

        :return: List of all joint names in the robot description
        :rtype: list of str
        """
        return self._all_joint_names

    def get_link_names(self):
        """
        Get list with the names of all link in the robot description.

        :return: List of all link names in the robot description
        :rtype: list of str
        """
        return self._all_link_names

    def _get_joint_info(self):
        """
        Get a dicts with all the information obtained from pybullets getJointInfo method for all joints.

        :return: getJointInfo() method return values from pybullet for all joints
        :rtype: list of dict
        """
        attribute_list = ['jointIndex', 'jointName', 'jointType',
                          'qIndex', 'uIndex', 'flags',
                          'jointDamping', 'jointFriction', 'jointLowerLimit',
                          'jointUpperLimit', 'jointMaxForce', 'jointMaxVelocity', 'linkName',
                          'jointAxis', 'parentFramePos', 'parentFrameOrn', 'parentIndex']

        joint_information = []
        for idx in self._all_joints:
            info = pb.getJointInfo(self._id, idx, physicsClientId=self._uid)
            joint_information.append(dict(zip(attribute_list, info)))
        return joint_information

    def get_joint_dict(self):
        """
        Get a dict with all joint names mapping to their joint index in the robot description.

        :return: Dict with all joint names and corresponding index
        :rtype: dict[str, int]
        """
        return self._all_joint_dict

    def get_joint_index_by_name(self, joint_name):
        """
        Get a joint index by the joint's name.
        :param joint_name: Name of joint
        :type joint_name: str

        :return: Joint index of given joint in robot description.
        :rtype: int
        """
        if joint_name in self._all_joint_dict:
            return self._all_joint_dict[joint_name]
        else:
            raise Exception("Joint name does not exist!")

    def get_link_dict(self):
        """
        Get a dict with all link names mapping to their link index in the robot description.

        :return: Dict with all link names and corresponding index
        :rtype: dict[str, int]
        """
        return self._all_link_dict

    def get_link_index_by_name(self, link_name):
        """
        Get a link index by the link's name.
        :param link_name: Name of link
        :type link_name: str

        :return: Link index of given link in robot description.
        :rtype: int
        """
        if link_name in self._all_link_dict:
            return self._all_link_dict[link_name]
        else:
            raise Exception("Link name does not exist!")

    def _find_movable_joints(self):
        """
        Get joint indices of all movable joints.

        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        movable_joints = []
        for i in self._all_joints:
            joint_info = pb.getJointInfo(
                self._id, i, physicsClientId=self._uid)
            # all moveable joints have type bigger than 0, -1 is a fixed joint
            if joint_info[3] > -1:
                movable_joints.append(i)
        return movable_joints

    def get_movable_joints(self):
        """
        Get joint indices of all movable joints in the robot description.
        :return: Indices of all movable joints in the robot description.
        :rtype: list of int
        """
        return self._movable_joints

    def get_nb_movable_joints(self):
        """
        Get number of movable joints in the robot description.

        :return: Number of movable joints in the robot description.
        :rtype: int
        """
        return self._nb_movable_joints

    def get_ft_sensor_joints(self):
        """
        Get indices of all joints with a force torque sensor activated.

        :return: Joint indices of joints with force torque sensors in the robot description. Returns False if
                 no joints have a FT sensor activated.
        :rtype: list of int
        """
        if hasattr(self, "_ft_joints"):
            return self._ft_joints
        else:
            print("No joints with FT sensor defined.")
            return False

    def _use_last_defined_link(self):
        """
        Get index and name of last link in the robot description.

        :return: Index and name of last link of the robot.
        :rtype: link_idx: int, link_name: str
        """
        joint_information = pb.getJointInfo(
            self._id, self._all_joints[-1], physicsClientId=self._uid)
        # joint_information[-1] is index of parent link, therefore add 1 to obtain child link index
        return joint_information[-1] + 1, joint_information[-5].decode("utf-8")

    def _find_joint_position_limits(self):
        """
        Find joint position limits, mean positions, and range for all movable joints.

        :return: Joint position limits, mean positions, and range for all movable joints.
        :rtype: dict[str, list of float]
        """
        lower_lim = np.zeros(self._nb_movable_joints)
        upper_lim = np.zeros(self._nb_movable_joints)
        mean_ = np.zeros(self._nb_movable_joints)
        range_ = np.zeros(self._nb_movable_joints)

        for k, idx in enumerate(self._movable_joints):
            lower_lim[k] = pb.getJointInfo(
                self._id, idx, physicsClientId=self._uid)[8]
            upper_lim[k] = pb.getJointInfo(
                self._id, idx, physicsClientId=self._uid)[9]
            mean_[k] = 0.5 * (lower_lim[k] + upper_lim[k])
            range_[k] = (upper_lim[k] - lower_lim[k])
        return {'lower': lower_lim, 'upper': upper_lim, 'mean': mean_, 'range': range_}

    def get_joint_position_limits(self):
        """
        Get joint positions limits, mean positions, and range for all movable joints.

        :return: Joint position limits, mean positions, and range for all movable joints.
        :rtype: dict[str, list of float]
        """
        return self._joint_position_limits

    def get_joint_velocity_limits(self):
        """
        Get joint velocity limits for all movable joints.

        :return: Joint velocity limits for all movable joints.
        :rtype: list of float
        """
        return self._joint_velocity_limits

    def get_joint_effort_limits(self):
        """
        Get joint effort limits for all movable joints.

        :return: Joint effort limits for all movable joints.
        :rtype: list of float
        """
        return self._joint_effort_limits

    def get_joint_limits(self):
        """
        Get joint position, velocity, and effort limits for all movable joints.

        :return: Joint position, velocity and effort limits for all movable joints.
        :rtype: list of dict
        """
        return self._joint_limits

    def set_ft_sensor_at(self, joint_id, enable=True):
        """
        Enable/Disable force/torque sensor at a desired joint
        :param joint_id: Desired joint index
        :param enable: Enable or disable FT sensor
        :type joint_id: int
        :type enable: bool | True

        :return: Boolean if action was successful or not
        :rtype: bool
        """
        if joint_id not in self._all_joints:
            print("The desired joint doesn't exist in the robot description.")
            return False
        if joint_id in self._ft_joints and enable:
            pass
        elif joint_id in self._ft_joints and not enable:
            self._ft_joints.remove(joint_id)
        elif joint_id not in self._ft_joints and enable:
            self._ft_joints.append(joint_id)
        elif joint_id not in self._ft_joints and not enable:
            print("There is no FT at the desired joint that could be disabled.")
            return False
        print("FT sensor at joint", joint_id)
        pb.enableJointForceTorqueSensor(self._id, joint_id, enable, self._uid)
        return True

    def set_default_joint_positions(self, joint_positions):
        """
        Set default joint positions for all movable joints.

        :param joint_positions: default joint positions of movable joints
        :type joint_positions: list of float

        :return: Boolean if action was successful
        :rtype: bool
        """
        if len(joint_positions) is not self._nb_movable_joints:
            print("Could not set default joint positions. Number of elements incorrect.")
            return False
        else:
            self._default_joint_positions = joint_positions
            return True

    def get_default_joint_positions(self):
        """
        Get default joint positions of all movable joints.

        :return: Default joint positions or False if they have not been set yet
        :rtype: list of float
        """
        if not hasattr(self, "_default_joint_positions"):
            print("Default joint positions have not been set yet.")
            return False
        else:
            return self._default_joint_positions

    def _set_base_pose(self):
        """
        Set position and orientation of the CoM of the robot base.
        """
        pb.resetBasePositionAndOrientation(
            self._id, self._base_pose['position'], self._base_pose['orientation'], physicsClientId=self._uid)

    def get_base_pose(self):
        """
        :return: Position [X,Y,Z] and orientation [X,Y,Z,W] of robot base.
        :rtype: list[float], list[float]
        """
        return self._base_pose['position'], self._base_pose['orientation']
