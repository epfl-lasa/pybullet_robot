import numpy as np
from pybullet_simulation import Simulation
from pybullet_simulation.worlds import EmptyWorld
from pybullet_robots.panda import PandaArm
from pybullet_controllers import OSImpedanceController
import time
import os

if __name__ == "__main__":
    simulation = Simulation(realtime_sim=False)
    simulation.add_PyB_models_path()

    world = EmptyWorld(simulation.uid, add_plane=True)
    world.add_object_from_urdf('table', 'table/table.urdf', position_xyz=[0.4, 0., 0.0],
                               orientation_wxyz=[0, 0, -0.707, 0.707], scaling=0.5, fixed_base=True)

    robot = PandaArm(robot_description=os.path.join(os.path.dirname(__file__), os.pardir, "models/panda_arm.urdf"),
                     uid=simulation.uid)

    slow_rate = 100.

    state = robot.get_state()
    goal_pos, goal_ori = state['ee_position'], state['ee_orientation']

    controller = OSImpedanceController(robot)
    robot.enable()

    print("started")

    z_traj = np.linspace(goal_pos[2], 0.34, 500)

    controller.start_controller_thread()

    i = 0

    while i < z_traj.size:
        now = time.time()

        state = robot.get_state()
        ee_pos = state['ee_position']

        goal_pos[2] = z_traj[i]

        print("Goal:", goal_pos, "Actual:", ee_pos)
        controller.update_goal(goal_pos, goal_ori)

        elapsed = time.time() - now
        sleep_time = (1. / slow_rate) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)

        i += 1

    controller.stop_controller_thread()
