import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .ros_interface import ROSInterface
from robots import BulletRobotState
import threading


class FrankaROSInterface(object):
    def __init__(self):
        self.interface = ROSInterface()
        self._joint_state_publisher = self.interface.add_publisher("/joint_states", JointState, 10)
        self.interface.add_subscriber_callback("/torque_controller/command", Float64MultiArray, self._command_callback)
        self._seq = 0
        self._mutex = threading.Lock()
        self._latest_command = []

    def is_connected(self):
        return self.interface.is_connected()

    def _command_callback(self, msg):
        with self._mutex:
            self._latest_command = msg.data

    def publish_robot_state(self, state):
        """
        Publish robot state to ZMQ socket.

        :param state: Current robot state as defined in bullet_robot.py
        :type state: BulletRobotState

        :return: Boolean if sending was successful
        :rtype: bool
        """
        message = JointState()
        message.name = state.joint_names
        message.position = state.joint_positions
        message.velocity = state.joint_velocities
        message.effort = state.joint_efforts
        message.header.stamp = rospy.Time.now()
        message.header.seq = self._seq
        self._seq = self._seq + 1
        self._joint_state_publisher.publish(message)

    def get_command(self):
        with self._mutex:
            return self._latest_command if len(self._latest_command) else False
