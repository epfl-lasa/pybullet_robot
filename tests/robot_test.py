from pybullet_simulation import Simulation
from pybullet_simulation.worlds import EmptyWorld
from pybullet_robots import BulletRobot
import time
import os.path

if __name__ == "__main__":
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
    robot = BulletRobot(robot_urdf="panda_arm.urdf", enforce_joint_limits=True, uid=simulation.uid)
    # Test some bullet_robot_description methods
    robot.set_default_joint_positions([-0.017792060227770554, -0.7601235411041661, 0.019782607023391807,
                                       -2.342050140544315, 0.029840531355804868, 1.5411935298621688,
                                       0.7534486589746342])
    robot.set_enforce_joint_limits(False)
    # print(robot.get_link_index_by_name('link')) # this raises an exception
    print(robot.get_joint_index_by_name('panda_joint5'))
    print(robot.get_joint_limits())
    print(robot.get_default_joint_positions())

    # Test some bullet_robot methods
    robot.goto_default_joint_positions()
    print(robot.disable())
    print(robot.set_control_mode('test'))
    print(robot.get_joint_positions())
    print(robot.set_control_mode('position'))

    robot.enable()
    print(robot.goto_default_joint_positions())
    time.sleep(2)
    cmd_velocities = [0.1, 0, 0, 0, 0, 0, 0]
    cmd_positions = [-0.017792060227770554, -0.7601235411041661, 0.519782607023391807, -2.342050140544315,
                     0.029840531355804868, 1.9411935298621688, 0.7534486589746342]
    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        if robot.enabled:
            state = robot.get_state()
            if k < 2000 and robot.check_robot_state(state):
                robot.set_joint_positions_cmd(cmd_positions)
            elif k == 2000 and robot.check_robot_state(state):
                robot.disable()
                robot.goto_default_joint_positions()
                robot.set_control_mode('velocity')
                robot.enable()
                robot.set_enforce_joint_limits(True)
            elif k > 2000 and robot.check_robot_state(state):
                robot.set_joint_velocities_cmd(cmd_velocities)
        simulation.step()

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1
        # print("Average rate: ", k / (time.time() - start))
