import numpy as np
from simulation import Simulation
from simulation.worlds import EmptyWorld
from robots.panda import PandaArm
import time
from interfaces import FrankaZMQInterface
import os.path

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = FrankaZMQInterface()

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
    while simulation.is_alive() and interface.is_connected():
        now = time.time()
        state = robot.get_state()

        # set unique values in all fields to check integrity of the communication because it's
        # important to check that conventions (column-major/row-major, quaternion, twist
        # force/torque) are the same in the simulation interface (python) and the control (cpp)
        state.joint_positions = np.ones((robot.get_nb_movable_joints(),))
        state.joint_velocities = 2 * np.ones((robot.get_nb_movable_joints(),))
        state.joint_efforts = 3 * np.ones((robot.get_nb_movable_joints(),))
        state.ee_position = 4 * np.ones((3,))
        state.ee_orientation.w = 1.0
        state.ee_orientation.y = 0.5
        state.ee_force = 5 * np.ones((3,))
        state.ee_torque = 6 * np.ones((3,))
        state.ee_linear_velocity = 7 * np.ones((3,))
        state.ee_angular_velocity = 8 * np.ones((3,))
        state.jacobian[1, 0] = 250.0
        state.jacobian[3, 5] = 250.0
        state.jacobian[5, 6] = 250.0
        state.inertia[1, 0] = 250.0
        state.inertia[3, 5] = 170.0
        state.inertia[5, 6] = 10.0

        interface.publish_robot_state(state)
        command = interface.get_command(robot.get_nb_movable_joints())
        if command:
            print(command)
        # if interface.first_message_received:
        #     if interface.timeout_triggered:
        #         # TODO handle connection timeout
        #         pass
        #     else:
        #         print(command)

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1

        # print("Average rate: ", k / (time.time() - start))
