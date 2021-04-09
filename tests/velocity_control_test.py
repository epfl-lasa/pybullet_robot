from simulation import Simulation
from simulation.worlds import EmptyWorld
from robots import BulletRobot
import time
from interfaces.franka_zmq_simulation_interface import FrankaZMQSimulationInterface
import os.path

if __name__ == "__main__":
    # create interface with default state_uri and command_uri
    interface = FrankaZMQSimulationInterface()

    desired_frequency = 500.0
    # create simulation object
    simulation = Simulation(realtime_sim=False, realtime_sim_freq=desired_frequency)

    # load empty world and add table
    world = EmptyWorld(uid=simulation.uid, add_plane=True, gravity=[0, 0, -9.81])
    world.add_object_from_urdf('table', 'table/table.urdf', position_xyz=[0.4, 0, 0],
                               orientation_wxyz=[0, 0, -0.707, 0.707], fixed_base=True, scaling=0.5)

    # create robot after adding models path
    if not simulation.add_robot_models_path(os.path.join(os.path.dirname(__file__), os.pardir, "models")):
        exit(1)
    robot = BulletRobot(robot_urdf="panda_arm.urdf", enforce_joint_limits=False, uid=simulation.uid)
    # Test some bullet_robot_description methods
    robot.set_default_joint_positions([-0.017792060227770554, -0.7601235411041661, 0.019782607023391807,
                                       -2.342050140544315, 0.029840531355804868, 1.5411935298621688,
                                       0.7534486589746342])
    robot.goto_default_joint_positions()
    robot.set_control_mode('velocity')
    robot.enable()

    time.sleep(1)
    command = [0, 0, 0, 0, 0, 0, 0]
    first_command_received = False
    recent_command_time = 0.0
    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        state = robot.get_state()

        interface.send(state)
        interface.poll_command()
        if interface.first_message_received:
            if interface.timeout_triggered:
                robot.disable()
            else:
                robot.set_joint_velocities_cmd(interface.current_command)

        simulation.step()

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1

        # print("Average rate: ", k / (time.time() - start))
