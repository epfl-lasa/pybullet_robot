import numpy as np
import pybullet as pb
from pybullet_simulation.worlds import SimpleWorld, add_PyB_models_to_path
from pybullet_robots.panda import PandaArm
import time
from pybullet_interfaces.franka_zmq_simulation_interface import FrankaZMQSimulationInterface
import os.path

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = FrankaZMQSimulationInterface()

    robot = PandaArm(robot_description=os.path.join(os.path.dirname(__file__), os.pardir, "models/panda_arm.urdf"))
    # print(robot._uid)
    # print(robot._id)
    # print(robot._rt_sim)
    # print(robot._all_joints)
    # print(robot._movable_joints)
    # print(robot._nu)
    # print(robot._nq)
    # print(robot._all_joint_names)
    # print(robot._all_joint_dict)
    # print(robot._ee_link_idx)
    # print(robot._ee_link_name)
    # print(robot._joint_limits)
    # print(robot._ft_joints)

    add_PyB_models_to_path()

    plane = pb.loadURDF('plane.urdf')
    table = pb.loadURDF('table/table.urdf',
                        useFixedBase=True, globalScaling=0.5)
    pb.resetBasePositionAndOrientation(
        table, [0.4, 0., 0.0], [0, 0, -0.707, 0.707])

    objects = {'plane': plane,
               'table': table}

    world = SimpleWorld(robot, objects)

    desired_frequency = 500.

    start = time.time()
    k = 0
    while True:
        now = time.time()
        state = world.robot.state()

        # set unique values in all fields to check integrity of the communication because it's
        # important to check that conventions (column-major/row-major, quaternion, twist
        # force/torque) are the same in the simulation interface (python) and the control (cpp)
        for i, key in enumerate(state):
            if key is 'ee_ori':
                state[key].w = 1
                state[key].x = 0
                state[key].y = 0.5
                state[key].z = 0
            elif key is 'tip_state':
                for j, key2 in enumerate(state[key]):
                    state[key][key2] = (j + 1) * i * np.ones(state[key][key2].shape)
                state[key]['force'][1] = 100.0
                state[key]['torque'][2] = -100.0
            else:
                state[key] = i * np.ones(state[key].shape)
                if key is 'jacobian':
                    state[key][1, 0] = 200.0
                    state[key][3, 3] = 150.0
                    state[key][4, 6] = 100.0
                if key is 'inertia':
                    state[key][1, 0] = 250.0
                    state[key][3, 5] = 170.0
                    state[key][5, 6] = 10.

        interface.send(state)
        command_message = interface.poll()
        if command_message:
            command = interface.get_command(command_message)
            print(command)

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1

        print("Average rate: ", k / (time.time() - start))
