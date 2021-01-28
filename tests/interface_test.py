import numpy as np
from pybullet_simulation import Simulation
from pybullet_simulation.worlds import EmptyWorld
from pybullet_robots.panda import PandaArm
import time
from pybullet_interfaces.franka_zmq_simulation_interface import FrankaZMQSimulationInterface
import os.path

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = FrankaZMQSimulationInterface()

    simulation = Simulation(realtime_sim=False)
    simulation.add_PyB_models_path()

    world = EmptyWorld(simulation.uid, add_plane=True)
    world.add_object_from_urdf('table', 'table/table.urdf', position_xyz=[0.4, 0., 0.0],
                               orientation_wxyz=[0, 0, -0.707, 0.707], scaling=0.5, fixed_base=True)

    robot = PandaArm(robot_description=os.path.join(os.path.dirname(__file__), os.pardir, "models/panda_arm.urdf"),
                     uid=simulation.uid)

    print(robot._uid)
    print(robot._id)
    print(robot._all_joints)
    print(robot._movable_joints)
    print(robot._nb_movable_joints)
    print(robot._all_joint_names)
    print(robot._all_joint_dict)
    print(robot._ee_link_idx)
    print(robot._ee_link_name)
    print(robot._joint_limits)
    print(robot._ft_joints)

    desired_frequency = 500.

    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        state = robot.get_state()

        # set unique values in all fields to check integrity of the communication because it's
        # important to check that conventions (column-major/row-major, quaternion, twist
        # force/torque) are the same in the simulation interface (python) and the control (cpp)
        for i, key in enumerate(state):
            if key == 'ee_ori':
                state[key].w = 1
                state[key].x = 0
                state[key].y = 0.5
                state[key].z = 0
            elif key == 'tip_state':
                for j, key2 in enumerate(state[key]):
                    state[key][key2] = (j + 1) * i * np.ones(state[key][key2].shape)
                state[key]['force'][1] = 100.0
                state[key]['torque'][2] = -100.0
            else:
                state[key] = i * np.ones(state[key].shape)
                if key == 'jacobian':
                    state[key][1, 0] = 200.0
                    state[key][3, 3] = 150.0
                    state[key][4, 6] = 100.0
                if key == 'inertia':
                    state[key][1, 0] = 250.0
                    state[key][3, 5] = 170.0
                    state[key][5, 6] = 10.

        interface.send(state)
        command = interface.poll_command()
        if interface.first_message_received:
            if interface.timeout_triggered:
                # TODO handle connection timeout
                pass
            else:
                print(command)

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1

        print("Average rate: ", k / (time.time() - start))
