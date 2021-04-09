import struct
import traceback
import time

import quaternion
import zmq


class FrankaZMQSimulationInterface(object):
    """
    Franka ZMQ Simulation Interface for communication between the Franka controller stack and the Franka simulation.
    This is the python implementation of the franka_lightweight_interface
    (https://github.com/epfl-lasa/franka_lightweight_interface).

    Available methods (for usage, see documentation at function definition):
        - send
        - receive
        - poll_command
    """

    def __init__(self, state_uri="0.0.0.0:5550", command_uri="0.0.0.0:5551", command_timeout=0.5, datatype='d'):
        """
        Constructor of the FrankaZMQSimulationInterfrace class, binds and publishes to the ZMQ sockets defined by
        their URIs.

        :param state_uri: URI of the socket over which the state is transmitted
        :param command_uri: URI of the socket over which the command is transmitted
        :param command_timeout: Acceptable time interval between two received commands before the interface considers
                                the transmission to be interrupted
        :param datatype: Data type on the C++ side

        :type state_uri: str
        :type command_uri: str
        :type command_timeout: float
        :type datatype: str
        """
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://" + state_uri)

        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.subscriber.setsockopt(zmq.CONFLATE, 1)
        self.subscriber.connect("tcp://" + command_uri)

        self.first_message_received = False
        self.current_command = [0] * 7  # TODO what is a good 'zero' command
        self.timeout_triggered = False
        self._command_timeout = command_timeout
        self._last_command = time.time()

        if datatype == 'd':
            self.datatype = datatype
        else:
            raise ValueError('[FrankaZMQSimulationInterface] The desired datatype is not implemented yet.')

    def send(self, state):
        """
        Send state to ZMQ socket.

        :param state: Current robot state as defined in bullet_robot.py
        :type state: dict[str, list of float]

        :return: Boolean if sending was successful
        :rtype: bool
        """
        encoded_state = self._encode_state(self._get_state_as_list(state), self.datatype)
        res = self.publisher.send(encoded_state, flags=0)
        return res is None

    def receive(self, flags=0):
        """
        Receive message from ZMQ socket.

        :param flags: ZMQ flags
        :type flags: int

        :return: Message if message was received, False if there was an error
        """
        try:
            message = self.subscriber.recv(flags=flags)
            return message
        except zmq.ZMQError as e:
            if e.errno is not zmq.EAGAIN:
                traceback.print_exc()
            return False

    def _poll(self):
        """
        Receive message from ZMQ socket with flag NOBLOCK.

        :return: Message if message was received, False if there was no new message received
        """
        return self.receive(flags=zmq.NOBLOCK)

    def poll_command(self):
        """
        Receive message from ZMQ socket and decode it into a command message. If no new messages have been received
        over a defined time horizon, a timeout is triggered. The command is available at self.current_command and the
        timeout flag at self.timeout_triggered.

        :return: Current command
        :rtype: list of float
        """
        message = self._poll()
        if message:
            if not self.first_message_received:
                self.first_message_received = True
            self.current_command = self._decode_command(message)
            self.timeout_triggered = False
            self._last_command = time.time()
        elif self.first_message_received and time.time() - self._last_command > self._command_timeout:
            self.current_command = [0] * 7
            self.timeout_triggered = True
        return self.current_command

    @staticmethod
    def _get_state_as_list(state=None):
        """
        Transform state dictionary to a list of floats.

        :param state: Current robot state as defined in bullet_robot.py
        :type state: dict[str, list of float]

        :return: State as list
        :rtype: list of float
        """
        state_list = []
        state_list.extend(list(state['joint_positions']))
        state_list.extend(list(state['joint_velocities']))
        state_list.extend(list(state['joint_torques']))
        state_list.extend(list(state['ee_position']))
        state_list.extend(quaternion.as_float_array(state['ee_orientation']))
        state_list.extend(list(state['ee_linear_velocity']))
        state_list.extend(list(state['ee_angular_velocity']))
        state_list.extend(list(state['ee_force']))
        state_list.extend(list(state['ee_torque']))
        state_list.extend(state['jacobian'].flatten('F'))  # F for column-major
        state_list.extend(state['inertia'].flatten('F'))  # F for column-major
        return state_list

    def _decode_command(self, command_msg):
        """
        Decode message from ZMQ socket.

        :param command_msg: Message as received from ZMQ socket
        :type command_msg: list of bytes

        :return: Decoded message as list of floats
        :rtype: list of float
        """
        if self.datatype == 'd':
            block_size = 8
        else:
            return None
        return [struct.unpack(self.datatype, command_msg[i:i + block_size])[0] for i in
                range(0, len(command_msg), block_size)]

    @staticmethod
    def _encode_state(state_list, datatype):
        """
        Encode state to send it over ZMQ socket.

        :param state_list: State as list of floats
        :param datatype: Data type used by CPP side
        :type state_list: list of float
        :type datatype: str

        :return: State as list of bytes
        :rtype: list of bytes
        """
        return b"".join([struct.pack(datatype, state_list[i]) for i in range(len(state_list))])
