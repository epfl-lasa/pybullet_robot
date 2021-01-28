import pybullet as pb
import numpy as np
import quaternion

from .bullet_robot_description import BulletRobotDescription


class BulletRobot(BulletRobotDescription):
    """
    Bullet Robot extending from Bullet Robot Description (tested only with the Franka Panda).
    This is a collection of methods that gather 'real-time' information about the robot state and take control over the
    robot. In other words, these are the methods called (at each time step) in the simulation loop. Additionally, a few
    high level methods for enabling, disabling, and emergency stopping to simulate a real robot (if desired) are
    implemented.

    Available methods (for usage, see documentation at function definition):
        - set_enforce_joint_limits
        - enable
        - disable
        - check_robot_state
        - get_state
        - get_joint_positions
        - get_joint_velocities
        - get_joint_efforts
        - get_jacobian
        - get_inertia
        - get_joint_state
        - get_ft_sensor_wrench
        - get_link_state
        - get_ee_state
        - set_control_mode
        - reset_joint_positions
        - set_joint_positions_cmd
        - set_joint_velocities_cmd
        - set_joint_torques_cmd
        - goto_default_joint_positions
    """

    def __init__(self, robot_urdf, enforce_joint_limits, uid, config=None):
        """
        Constructor of the Robot class.

        :param robot_urdf: robot description file (urdf, .bullet, etc.)
        :param enforce_joint_limits: Enforce joint limits or not
        :param uid: server id of PyBullet
        :param config: optional config file for specifying robot information

        :type robot_urdf: str
        :type enforce_joint_limits: bool
        :type uid: int
        :type config: dict
        """
        BulletRobotDescription.__init__(self, robot_urdf=robot_urdf, uid=uid, config=config)
        self._enforce_joint_limits = enforce_joint_limits
        self.enabled = False

    def set_enforce_joint_limits(self, enforce_joint_limits):
        """
        Set boolean to enforce joint position, velocity, and effort limits during simulation.

        :param enforce_joint_limits: Enforce joint limits or not
        :type enforce_joint_limits: bool
        """
        self._enforce_joint_limits = enforce_joint_limits

    def enable(self):
        """
        Enable robot.

        :return: Robot status (enabled/disabled)
        :rtype: bool
        """
        if self.check_robot_state(self.get_state()):
            print("Robot enabled")
            self.enabled = True
        else:
            print("Couldn't enable robot")
            self.enabled = False
        return self.enabled

    def disable(self):
        """
        Disable robot.

        :return: Robot status (enabled/disabled)
        :rtype: bool
        """
        if self.enabled:
            print("Robot disabled")
        self._emergency_stop("", verbose=False)
        return self.enabled

    def _emergency_stop(self, msg="no reason specified", verbose=True):
        """
        Emergency stop of robot, put zero velocity in all joints.

        :param msg: Reason for emergency stop
        :param verbose: optional parameter to specify if message should be printed
        :type msg: str
        :type verbose: bool
        """
        self.enabled = False
        pb.setJointMotorControlArray(
            self._id, self._movable_joints, controlMode=pb.VELOCITY_CONTROL,
            targetVelocities=[0] * self._nb_movable_joints, physicsClientId=self._uid)
        if verbose:
            print("Emergency stop! ", msg)

    def check_robot_state(self, state):
        """
        Check if the current robot state is within the joint limits.

        :param state: State of the robot
        :type: state: dict

        :return ok: Boolean if robot state is okay
        :rtype ok: bool
        """
        ok = False
        if not all([lower_pos_lim <= joint_pos <= upper_pos_lim for lower_pos_lim, upper_pos_lim, joint_pos in
                    zip(self._joint_position_limits['lower'], self._joint_position_limits['upper'],
                        state['joint_pos'])]):
            message = "Joint position limits"
        elif any([abs(joint_vel) >= vel_limit for joint_vel, vel_limit in
                  zip(state['joint_vel'], self._joint_velocity_limits)]):
            message = "Joint velocity limits"
        elif any([abs(joint_effort) >= effort_limit for joint_effort, effort_limit in
                  zip(state['joint_eff'], self._joint_effort_limits)]):
            message = "Joint effort limits"
        else:
            return True
        if not ok and not self._enforce_joint_limits:
            print("WARNING: ", message)
        if not ok and self._enforce_joint_limits:
            self._emergency_stop(message)

    def get_state(self):
        """
        :return: Current robot state, as a dictionary, containing
                joint positions, velocities, efforts, jacobian,
                joint space inertia tensor, end-effector position,
                end-effector orientation, end-effector velocity (linear and angular),
                end-effector force, end-effector torque
        :rtype: dict: {'joint_pos': np.ndarray,
                       'joint_vel': np.ndarray,
                       'joint_eff': np.ndarray,
                       'jacobian': np.ndarray,
                       'inertia': np.ndarray,
                       'ee_pos': np.ndarray,
                       'ee_ori': np.ndarray,
                       'ee_vel': np.ndarray,
                       'ee_omg': np.ndarray,
                       'tip_state'['force']: np.ndarray,
                       'tip_state'['torque']: np.ndarray,
                       }
        """

        state = {}
        state['joint_pos'], state['joint_vel'], _, state['joint_eff'] = self.get_joint_state()
        state['jacobian'] = self.get_jacobian(state['joint_pos'])
        state['inertia'] = self.get_inertia(state['joint_pos'])

        state['ee_pos'], state['ee_ori'], state['ee_vel'], state['ee_omg'] = self.get_ee_state()

        tip_state = {}
        if hasattr(self, "_ft_joints"):
            ft_joint_state = pb.getJointState(self._id, max(
                self._ft_joints), physicsClientId=self._uid)
            ft = np.asarray(ft_joint_state[2])
        else:
            ft = [0.0] * 6

        tip_state['force'] = ft[:3]
        tip_state['torque'] = ft[3:]

        state['tip_state'] = tip_state

        return state

    def get_joint_positions(self):
        """
        Get current joint positions of movable joints.

        :return: Current joint positions of movable joints.
        :rtype: list of float
        """
        return self.get_joint_state()[0].tolist()

    def get_joint_velocities(self):
        """
        Get current joint velocities of movable joints.

        :return: Current joint velocities of movable joints
        :rtype: list of float
        """
        return self.get_joint_state()[1].tolist()

    def get_joint_efforts(self):
        """
        Get current joint efforts of movable joints.

        :return: Current joint efforts of movable joints
        :rtype: list of float
        """
        return self.get_joint_state()[3].tolist()

    def get_jacobian(self, joint_positions):
        """
        Compute jacobian for given joint positions.

        :param joint_positions: The returned jacobian will be evaluated at given joint positions
        :type joint_positions: numpy.ndarray

        :return: Jacobian matrix for current or provided joint configuration, of shape (6, DOF), or False if unsuccessful
        :rtype: numpy.ndarray
        """
        if len(joint_positions) is not self._nb_movable_joints:
            print("Invalid number of elements in your input.")
            return False
        else:
            linear_jac, angular_jac = pb.calculateJacobian(bodyUniqueId=self._id,
                                                           linkIndex=self._ee_link_idx,
                                                           localPosition=[0.0, 0.0, 0.0],
                                                           objPositions=joint_positions.tolist(),
                                                           objVelocities=np.zeros(self._nb_movable_joints).tolist(),
                                                           objAccelerations=np.zeros(self._nb_movable_joints).tolist(),
                                                           physicsClientId=self._uid)

        jacobian = np.vstack([np.array(linear_jac), np.array(angular_jac)])
        return jacobian

    def get_inertia(self, joint_positions):
        """
        Compute the inertia matrix for given joint positions.

        :param joint_positions: The returned inertia is evaluated at given joint positions
        :type joint_positions: numpy.ndarray

        :return: Inertia matrix for current or provided joint configuration, of shape (DOF, DOF), or False if unsuccessful
        :rtype: numpy.ndarray
        """
        if len(joint_positions) is not self._nb_movable_joints:
            print("Invalid number of elements in your input.")
            return False
        else:
            inertia_tensor = np.array(pb.calculateMassMatrix(self._id, joint_positions.tolist()))
        return inertia_tensor

    def get_joint_state(self, joint_id=None):
        """
        Get joint state(s) (position, velocty, force, effort).

        :param joint_id: Optional parameter, if different from None, then only the joint state of the desired joint is
                         returned (if it exists), otherwise the joint states of all joints are returned.
        :type joint_id: int

        :return: Joint positions, velocities, reaction forces, and efforts of all movable joints given by bullet physics
        :rtype: list of numpy.ndarray
        """
        if joint_id is None:
            joint_positions = []
            joint_velocities = []
            joint_reaction_forces = []
            joint_efforts = []

            for idx in self._movable_joints:
                joint_state = pb.getJointState(
                    self._id, idx, physicsClientId=self._uid)
                joint_positions.append(joint_state[0])
                joint_velocities.append(joint_state[1])
                joint_reaction_forces.append(joint_state[2])
                joint_efforts.append(joint_state[3])
            return np.array(joint_positions), np.array(joint_velocities), np.array(joint_reaction_forces), np.array(
                joint_efforts)

        else:
            if joint_id in self._all_joints:
                joint_state = pb.getJointState(
                    self._id, joint_id, physicsClientId=self._uid)
                joint_positions = joint_state[0]
                joint_velocities = joint_state[1]
                joint_reaction_forces = joint_state[2]
                joint_efforts = joint_state[3]
                return joint_positions, joint_velocities, np.array(joint_reaction_forces), joint_efforts
            else:
                print("Desired joint index doesn't exist")
                return None

    def get_ft_sensor_wrench(self, ft_joint_index, in_world_frame=True, ft_link='child'):
        '''
        Get wrench of force torque sensor in world or local frame.

        :param in_world_frame: if True, computes reaction forces in local sensor frame, else in base frame of robot
        :param ft_link: One of ['child', 'parent']. Decide if child or parent link should be considered to transform
                        wrench into world frame.
        :type in_world_frame: bool
        :type ft_link: str

        :return: End effector forces and torques [fx, fy, fz, tx, ty, tz]
        :rtype: np.ndarray
        '''
        if not hasattr(self, "_ft_joints"):
            print("There are no joints with FT sensors defined.")
            return False
        if hasattr(self, "_ft_joints") and not ft_joint_index in self._ft_joints:
            print("No FT sensor at specified joint")
            return False
        if in_world_frame and ft_link not in ['child', 'parent']:
            print("Parameter ft_link has to be one of ['child', 'parent'].")
            return False

        _, _, joint_reaction_force, _ = self.get_joint_state(ft_joint_index)

        if in_world_frame:
            joint_reaction_force = np.asarray(joint_reaction_force)
            if ft_link == 'child':
                link_pos, link_ori, _, _ = self.get_link_state(ft_joint_index)
            else:
                link_pos, link_ori, _, _ = self.get_link_state(ft_joint_index - 1)
            rot_mat = quaternion.as_rotation_matrix(link_ori)
            f = np.dot(rot_mat,
                       np.asarray([-joint_reaction_force[0], -joint_reaction_force[1], -joint_reaction_force[2]]))
            t = np.dot(rot_mat,
                       np.asarray([-joint_reaction_force[3], -joint_reaction_force[4], -joint_reaction_force[5]]))
            joint_reaction_force = np.append(f, t).flatten()

        return joint_reaction_force

    def get_link_state(self, link_id):
        """
        Get state of desired link.

        :param link_id: Index of desired link
        :type link_id: int

        :return: State of link (cartesian position of link frame in robot description,
                                cartesian orientation of link frame in robot description in quaternion wxyz,
                                cartesian linear velocity, cartesian angular velocity)
        :rtype: list of numpy.ndarray
        """
        if link_id not in self._all_links:
            print("Link doesn't exists.")
            return False
        else:
            link_state = pb.getLinkState(self._id, link_id, computeLinkVelocity=1, physicsClientId=self._uid)
            pos = np.asarray(link_state[4])
            ori = np.quaternion(link_state[5][3], link_state[5][0], link_state[5][1],
                                link_state[5][2])  # hamilton convention
            lin_vel = np.asarray(link_state[6])
            ang_vel = np.asarray(link_state[7])

            return pos, ori, lin_vel, ang_vel

    def get_ee_state(self):
        """
        Get end-effector state.

        :return: End-effector link state of this robot in the format (position, orientation, linear velocity, angular
                 velocity). Note: Orientation is a quaternion following Hamilton convention, i.e. (w, x, y, z)
        :rtype: list of numpy.ndarray
        """
        return self.get_link_state(link_id=self._ee_link_idx)

    def set_control_mode(self, control_mode='position'):
        """
        Use to set the robot control mode.

        :param control_mode: Desired control mode, one of ['position','velocity','torque']
        :type control_mode: str

        :return: Success of action
        :rtype: bool

        """
        # TODO
        if self.enabled:
            print("Disable robot first to change control mode")
            return False

        if self._enforce_joint_limits:
            efforts = self._joint_effort_limits
        else:
            efforts = [500] * self._nb_movable_joints

        if control_mode == 'position':
            pb.setJointMotorControlArray(self._id, self._movable_joints, pb.POSITION_CONTROL,
                                         targetPositions=self.get_joint_positions(), forces=efforts,
                                         physicsClientId=self._uid)
        elif control_mode == 'velocity':
            pb.setJointMotorControlArray(self._id, self._movable_joints, pb.VELOCITY_CONTROL,
                                         targetVelocities=[0.0] * self._nb_movable_joints, forces=efforts,
                                         physicsClientId=self._uid)
        elif control_mode == 'torque':
            pb.setJointMotorControlArray(self._id, self._movable_joints, pb.VELOCITY_CONTROL,
                                         forces=[0.0] * self._nb_movable_joints, physicsClientId=self._uid)
        else:
            print("Invalid control mode, choose one of ['position', 'velocity', 'torque'].")
            return False
        return True

    def reset_joint_positions(self, joint_positions, joint_indices=None):
        """
        Reset joint positions of movable joints. Note: this will hard reset the joints, no controllers used.

        :param joint_positions: Joint position values
        :param joint_indices: Optional parameter, if different from None, then only the joint position of the specified
                              joints are changed. Otherwise all movable joints are considered.
        :type joint_positions: list of float
        :type joint_indices: list of int

        :return: Boolean if the action was successful.
        :rtype: bool
        """
        if self.enabled:
            print("Disable robot first to reset joint positions")
            return False

        if joint_indices is not None:
            if any([joint_id not in self._movable_joints for joint_id in joint_indices]):
                print("You specified at least one joint that doesn't exist or is not movable.")
                return False
        else:
            joint_indices = self._movable_joints

        if len(joint_positions) is not len(joint_indices):
            print("Your input variables must have the same length")
            return False
        if not all([lower_pos_lim <= joint_pos <= upper_pos_lim for lower_pos_lim, upper_pos_lim, joint_pos in
                    zip(self._joint_position_limits['lower'][joint_indices],
                        self._joint_position_limits['upper'][joint_indices],
                        joint_positions)]) and self._enforce_joint_limits:
            print("At least one of the desired values is outside joint limits")
            return False

        for i, joint_idx in enumerate(joint_indices):
            pb.resetJointState(self._id, joint_idx,
                               joint_positions[i], physicsClientId=self._uid)
        return True

    def set_joint_positions_cmd(self, joint_positions, joint_indices=None, kp=0.002, kd=1.0):
        """
        Set target joint positions for position control. This control method tries to minimize the error e =
        position_gain*(desired_position-actual_position)+velocity_gain*(desired_velocity-actual_velocity).

        :param joint_positions: desired joint positions
        :param joint_indices: Optional parameter, if different from None, then only the joint position of the specified
                              joints are changed. Otherwise all movable joints are considered.
        :param kp: position gain in error computation
        :param kd: velocity gain in error computation

        :type joint_positions: list of float
        :type joint_indices: list of int
        :type kp: float
        :type kd: float

        :return: Boolean if the operation was successful.
        :rtype: bool
        """
        joint_indices = self._check_joint_command(joint_positions, joint_indices)
        if not joint_indices:
            return False

        if not all([lower_pos_lim <= joint_pos <= upper_pos_lim for lower_pos_lim, upper_pos_lim, joint_pos in
                    zip(self._joint_position_limits['lower'][joint_indices],
                        self._joint_position_limits['upper'][joint_indices],
                        joint_positions)]) and self._enforce_joint_limits:
            print("At least one of the desired values is outside joint limits")
            return False

        vels = [0.005] * self._nb_movable_joints
        efforts = [500] * self._nb_movable_joints
        kp = [kp] * self._nb_movable_joints
        kd = [kd] * self._nb_movable_joints
        pb.setJointMotorControlArray(self._id, joint_indices, controlMode=pb.POSITION_CONTROL,
                                     targetPositions=joint_positions, targetVelocities=vels, positionGains=kp,
                                     velocityGains=kd, forces=efforts, physicsClientId=self._uid)
        return True

    def set_joint_velocities_cmd(self, joint_velocities, joint_indices=None):
        """
        Set joint velocities of movable joints.

        :param joint_velocities: Joint velocity values
        :param joint_indices: Optional parameter, if different from None, then only the joint velocities of the specified
                              joints are changed. Otherwise all movable joints are considered.
        :type joint_velocities: list of float
        :type joint_indices: list of int

        :return: Boolean if the operation was successful.
        :rtype: bool
        """
        joint_indices = self._check_joint_command(joint_velocities, joint_indices)
        if not joint_indices:
            return False

        if any(abs(joint_vel) > vel_limit for joint_vel, vel_limit in
               zip(joint_velocities, [self._joint_velocity_limits[joint_id] for joint_id in
                                      joint_indices])) and self._enforce_joint_limits:
            self._emergency_stop("Desired velocities higher than max velocity")
            return False

        if self._enforce_joint_limits:
            efforts = self._joint_effort_limits
        else:
            efforts = [500] * self._nb_movable_joints
        pb.setJointMotorControlArray(
            self._id, self._movable_joints, controlMode=pb.VELOCITY_CONTROL, targetVelocities=joint_velocities,
            forces=efforts, physicsClientId=self._uid)
        return True

    def set_joint_torques_cmd(self, joint_torques, joint_indices=None):
        """
        Set joint torques of movable joints.

        :param joint_torques: Joint torque values
        :param joint_indices: Optional parameter, if different from None, then only the joint torques of the specified
                              joints are changed. Otherwise all movable joints are considered.
        :type joint_torques: list of float
        :type joint_indices: list of int

        :return: Boolean if the operation was successful.
        :rtype: bool
        """
        joint_indices = self._check_joint_command(joint_torques, joint_indices)
        if not joint_indices:
            return False

        pb.setJointMotorControlArray(
            self._id, joint_indices, controlMode=pb.TORQUE_CONTROL, forces=joint_torques, physicsClientId=self._uid)
        return True

    def _check_joint_command(self, joint_command, joint_indices=None):
        """
        Check if joint command has correct lenght and joint indices.

        :param joint_command: Desired joint command
        :param joint_indices: Optional parameter specifing the commanded joints
        :type joint_command: list of float
        :type joint_indices: list of int

        :return: Boolean if joint command is valid
        :rtype: bool
        """
        if not self.enabled:
            print("Enable robot first.")
            return False

        if joint_indices is not None:
            if any([joint_id not in self._movable_joints for joint_id in joint_indices]):
                print("You specified at least one joint that doesn't exist or is not movable.")
                return False
        else:
            joint_indices = self._movable_joints

        if len(joint_command) is not len(joint_indices):
            print("Your input variables must have the same length")
            return False
        return joint_indices

    def goto_default_joint_positions(self):
        """
        Reset joint positions of movable joints to the default joint positions.

        :return: Boolean if action was successful
        :rtype: bool
        """
        if hasattr(self, "_default_joint_positions"):
            return self.reset_joint_positions(self._default_joint_positions)
        else:
            print("Default joint positions not set yet.")
            return False
