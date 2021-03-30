from collections import deque
import numpy as np
from pybullet_simulation import Simulation
from pybullet_simulation.worlds import EmptyWorld
from pybullet_robots.panda import PandaArm
from pybullet_controllers import OSHybridController
import time
import matplotlib.pyplot as plt
import threading
import os


def plot_thread():
    plt.ion()
    while True:
        plt.clf()
        plt.plot(fx_deque, 'r', label='x')
        plt.plot(fy_deque, 'g', label='y')
        plt.plot(fz_deque, 'b', label='z')
        plt.legend()
        plt.draw()
        plt.pause(0.000001)
        if done:
            break


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

    controller = OSHybridController(robot)
    robot.enable()

    print("started")

    z_traj = np.linspace(goal_pos[2], 0.3, 550)

    plot_t = threading.Thread(target=plot_thread)
    fx_deque = deque([0], maxlen=1000)
    fy_deque = deque([0], maxlen=1000)
    fz_deque = deque([0], maxlen=1000)

    controller.start_controller_thread()

    done = False
    plot_t.start()

    i = 0
    f_ctrl = True
    while i < z_traj.size:
        now = time.time()

        state = robot.get_state()
        ee_pos = state['ee_position']
        wrench = robot.get_ft_sensor_wrench(robot.get_ft_sensor_joints()[-1], in_world_frame=True, ft_link='child')
        # print wrench
        if abs(wrench[2]) >= 10.:
            break

        goal_pos[2] = z_traj[i]

        controller.update_goal(goal_pos, goal_ori)

        fx_deque.append(wrench[0])
        fy_deque.append(wrench[1])
        fz_deque.append(wrench[2])

        elapsed = time.time() - now
        sleep_time = (1. / slow_rate) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)

        i += 1
    else:
        print("Never reached force threshold for switching controller")
        f_ctrl = False

    if f_ctrl:

        print("Switching to force control along Z axis")
        y_traj = np.linspace(goal_pos[1], goal_pos[1] - 0.2, 400)

        controller.change_ft_directions([0, 0, 1, 0, 0, 0])
        target_force = -21.

        p_slider = pb.addUserDebugParameter('p_f', 0.1, 2., controller._P_ft[2, 2])
        i_slider = pb.addUserDebugParameter('i_f', 0.0, 100., controller._I_ft[2, 2])
        w_slider = pb.addUserDebugParameter('windup', 0.0, 100., controller._windup_guard[2, 0])

        i = 0
        while i < y_traj.size:
            now = time.time()

            ee_pos, _ = world.robot.ee_pose()
            wrench = world.robot.get_ee_wrench(local=False)
            # print wrench
            goal_pos[1] = y_traj[i]

            controller._P_ft[2, 2] = pb.readUserDebugParameter(p_slider)
            controller._I_ft[2, 2] = pb.readUserDebugParameter(i_slider)
            controller._windup_guard[2, 0] = pb.readUserDebugParameter(w_slider)

            controller.update_goal(
                goal_pos, goal_ori, np.asarray([0., 0., target_force]))

            fx_deque.append(wrench[0])
            fy_deque.append(wrench[1])
            fz_deque.append(wrench[2])

            elapsed = time.time() - now
            sleep_time = (1. / slow_rate) - elapsed
            if sleep_time > 0.0:
                time.sleep(sleep_time)

            if i < y_traj.size - 1:
                i += 1

    controller.stop_controller_thread()
    done = True
    plot_t.join()
