import struct
import time

import quaternion
from robots import BulletRobotState

from .zmq_interface import ZMQInterface


class FrankaZMQInterface(object):
    """
    Franka ZMQ Simulation Interface for communication between the Franka controller stack and the Franka simulation.
    This is the python implementation of the franka_lightweight_interface
    (https://github.com/epfl-lasa/franka_lightweight_interface).

    Available methods (for usage, see documentation at function definition):
        - send
        - receive
        - poll_command
    """

    def __init__(self, state_uri="0.0.0.0:1601", command_uri="0.0.0.0:1602", command_timeout=0.5):
        """
        Constructor of the FrankaZMQInterface class, binds and publishes to the ZMQ sockets defined by
        their URIs.

        :param state_uri: URI of the socket over which the robot state is transmitted
        :param command_uri: URI of the socket over which the command is transmitted
        :param command_timeout: Acceptable time interval between two received commands before the interface considers
                                the transmission to be interrupted

        :type state_uri: str
        :type command_uri: str
        :type command_timeout: float
        """
        # set up ZMQ interface
        self.state_uri = state_uri
        self.command_uri = command_uri
        self.zmq_interface = ZMQInterface()
        self.zmq_interface.add_publisher(self.state_uri)
        self.zmq_interface.add_subscriber(self.command_uri)

        # self.first_message_received = False
        # self.current_command = [0] * 7  # TODO what is a good 'zero' command
        # self.timeout_triggered = False
        # self._command_timeout = command_timeout
        # self._last_command = time.time()

        self.data_types = {'d': 8}

    def is_connected(self):
        return self.zmq_interface.is_connected()

    def publish_robot_state(self, state):
        """
        Publish robot state to ZMQ socket.

        :param state: Current robot state as defined in bullet_robot.py
        :type state: BulletRobotState

        :return: Boolean if sending was successful
        :rtype: bool
        """
        encoded_state = self._encode_message(self._get_state_as_list(state), 'd')
        return self.zmq_interface.send(self.state_uri, encoded_state)

    def get_command(self):
        """
        Receive message from ZMQ socket and decode it into a command message. If no new messages have been received
        over a defined time horizon, a timeout is triggered. The command is available at self.current_command and the
        timeout flag at self.timeout_triggered.

        :return: Current command
        :rtype: list of float
        """
        message = self.zmq_interface.poll(self.command_uri)
        return self._decode_message(message, 'd') if message else None
        # if message:
        #     return self._decode_message(message, 'd')
        # else:
        #     return False
        #     if not self.first_message_received:
        #         self.first_message_received = True
        #     self.current_command = self._decode_message(message, 'd')
        #     self.timeout_triggered = False
        #     self._last_command = time.time()
        # elif self.first_message_received and time.time() - self._last_command > self._command_timeout:
        #     self.current_command = [0] * 7
        #     self.timeout_triggered = True
        # return self.current_command

    def _decode_message(self, message, data_type):
        """
        Decode message from ZMQ socket.

        :param message: Message from ZMQ socket
        :param data_type: Data type of the message
        :type message: bytes
        :type data_type: str

        :return: Decoded message
        :rtype: Any
        """
        return [struct.unpack(data_type, message[i:i + self.data_types[data_type]])[0] for i in
                range(0, len(message), self.data_types[data_type])]

    @staticmethod
    def _encode_message(message, data_type):
        """
        Encode message to send it over ZMQ socket.

        :param message: list of floats
        :param data_type: data type of the message
        :type message: Any
        :type data_type: str

        :return: State as list of bytes
        :rtype: bytes
        """
        return b"".join([struct.pack(data_type, message[i]) for i in range(len(message))])

    @staticmethod
    def _get_state_as_list(state):
        """
        Transform state dictionary to a list of floats.

        :param state: Current robot state as defined in bullet_robot_state.py
        :type state: BulletRobotState

        :return: State as list
        :rtype: list of float
        """
        state_list = []
        state_list.extend(state.joint_positions)
        state_list.extend(state.joint_velocities)
        state_list.extend(state.joint_efforts)
        state_list.extend(state.ee_position)
        state_list.extend(quaternion.as_float_array(state.ee_orientation))
        state_list.extend(state.ee_linear_velocity)
        state_list.extend(state.ee_angular_velocity)
        state_list.extend(state.ee_force)
        state_list.extend(state.ee_torque)
        state_list.extend(state.jacobian.flatten('F'))  # F for column-major
        state_list.extend(state.inertia.flatten('F'))  # F for column-major
        return state_list
