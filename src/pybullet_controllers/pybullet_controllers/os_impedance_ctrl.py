import numpy as np
from pybullet_controllers.utils import quatdiff_in_euler
from pybullet_controllers.os_controller import OSControllerBase
from pybullet_controllers.ctrl_config import OSImpConfig


class OSImpedanceController(OSControllerBase):

    def __init__(self, robot, config=OSImpConfig, **kwargs):
        OSControllerBase.__init__(self, robot=robot, config=config, **kwargs)

    def update_goal(self, goal_pos, goal_ori):
        self._mutex.acquire()
        self._goal_pos = np.asarray(goal_pos).reshape([3, 1])
        self._goal_ori = np.asarray(goal_ori)
        self._mutex.release()

    def _compute_cmd(self):
        """
        Actual control loop. Uses goal pose from the feedback thread
        and current robot states from the subscribed messages to compute
        task-space force, and then the corresponding joint torques.
        """
        state = self._robot.get_state()
        curr_pos, curr_ori = state['ee_position'], state['ee_orientation']

        delta_pos = self._goal_pos - curr_pos.reshape([3, 1])
        delta_ori = quatdiff_in_euler(
            curr_ori, self._goal_ori).reshape([3, 1])
        # print goal_pos, curr_pos

        curr_vel, curr_omg = state['ee_linear_velocity'], state['ee_angular_velocity']
        # print self._goal_pos, curr_pos
        # Desired task-space force using PD law
        F = np.vstack([self._P_pos.dot(delta_pos), self._P_ori.dot(delta_ori)]) - \
            np.vstack([self._D_pos.dot(curr_vel.reshape([3, 1])),
                       self._D_ori.dot(curr_omg.reshape([3, 1]))])

        error = np.asarray([np.linalg.norm(delta_pos), np.linalg.norm(delta_ori)])

        J = self._robot.get_jacobian(state['joint_positions'])

        # joint torques to be commanded
        return np.dot(J.T, F).flatten(), error

    def _initialise_goal(self):
        state = self._robot.get_state()
        self.update_goal(state['ee_position'], state['ee_orientation'])
