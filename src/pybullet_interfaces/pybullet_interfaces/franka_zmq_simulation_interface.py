import zmq
import struct
import numpy as np
import quaternion
import traceback


class FrankaZMQSimulationInterface(object):

    def __init__(self, state_uri="0.0.0.0:5550", command_uri="0.0.0.0:5551", datatype='d'):
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.connect("tcp://" + state_uri)

        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.subscriber.setsockopt(zmq.CONFLATE, 1)
        self.subscriber.connect("tcp://" + command_uri)

        if datatype == 'd':
            self.datatype = datatype
        else:
            print("This datatype has to be implemented first.")
            exit(1)

    def send(self, state):
        # state_list = self._get_state_as_list(state)
        encoded_state = self._encode_state(self._get_state_as_list(state), self.datatype)
        # print(state)
        res = self.publisher.send(encoded_state, flags=0)
        return res is None

    def receive(self, flags=0):
        try:
            message = self.subscriber.recv(flags=flags)
            return message
        except zmq.ZMQError as e:
            if e.errno is not zmq.EAGAIN:
                traceback.print_exc()
            return False

    def poll(self):
        return self.receive(flags=zmq.NOBLOCK)

    @staticmethod
    def _get_state_as_list(state=None):
        state_list = []
        state_list.extend(list(state['position']))
        state_list.extend(list(state['velocity']))
        state_list.extend(list(state['effort']))
        state_list.extend(list(state['ee_point']))
        state_list.extend(quaternion.as_float_array(state['ee_ori']))
        state_list.extend(list(state['ee_vel']))
        state_list.extend(list(state['ee_omg']))
        state_list.extend(list(state['tip_state']['force']))
        state_list.extend(list(state['tip_state']['torque']))
        state_list.extend(state['jacobian'].flatten('F'))  # F for column-major
        state_list.extend(state['inertia'].flatten('F'))  # F for column-major
        return state_list

    def get_command(self, command_msg):
        if self.datatype == 'd':
            block_size = 8
        else:
            return None
        return [struct.unpack(self.datatype, command_msg[i:i + block_size])[0] for i in
                range(0, len(command_msg), block_size)]

    @staticmethod
    def _encode_state(state_list, datatype):
        return b"".join([struct.pack(datatype, state_list[i]) for i in range(len(state_list))])
