# PyBullet Simulation

## Usage

### Preparation

If you don't have `pip3` and `pipenv` installed on your machine yet, do so:

```bash
sudo apt install python3-pip
python3 -m pip install --user pipenv
```

Additionally, run the following command to add the user base's binary directory to your `PATH` environmental variable:

```bash
echo 'export PATH="${HOME}/.local/bin:$PATH"' >> ~/.bashrc
```

, or,

```bash
echo 'export PATH="${HOME}/.local/bin:$PATH"' >> ~/.zshrc
```

Then, restart your shell.

### Setup pipenv

To create the pipenv for this project, run

```bash
git clone git@github.com:epfl-lasa/pybullet_robot.git
cd pybullet_robot
pipenv install
```

### Running scripts

Run your scripts using the command

```bash
pipenv run [some command]
```

, e.g.

```bash
pipenv run demos/simple_ctrl_test.py
```

or if the `demos/simple_ctrl_test.py` is declared as script in the Pipfile,

```bash
pipenv run simple_ctrl_test
```

# PyBullet ROBOT

<!-- ![PyPI pyversions](https://img.shields.io/pypi/pyversions/ansicolortags.svg) -->

A generel Python interface class for robot simulations using [PyBullet](https://www.pybullet.org). Python API class to
control and monitor the robot in the simulation. An implementation of subclass is available which uses the Franka Emika
Panda robot as an example. The interface is written in the structure similar to the [_
panda_robot_](https://github.com/justagist/panda_robot) ROS package used for controlling the real robot. This allows for
direct transfer of code to the real robot (when using the [_panda_robot_](https://github.com/justagist/panda_robot) ROS
package).

A simple world interface (SimpleWorld) is also provided for creating or modifying objects in the simulated world.

The robot can be directly controlled using position, velocity, or torque control of joints. Example implementations of
task-space control (Impedance control, Hybrid force-motion control) using joint torque control is also available.

Although, this package is structured as a ROS package, it can be used without ROS.

## Dependencies

- `pip install -r requirements.txt` (numpy, quaternion, pybullet)

## Related Packages

- [_mujoco_panda_](https://github.com/justagist/mujoco_panda) : Simulation using Mujoco Physics Engine (pymujoco) with
  exposed controllers and state feedback. Also provides a Python interface class to control and monitor the robot in the
  simulation. The interface is written in the structure similar to the [_
  panda_robot_](https://github.com/justagist/panda_robot) ROS package used for controlling the real robot. This allows
  for direct transfer of code to the real robot (when using the [_
  panda_robot_](https://github.com/justagist/panda_robot) ROS package).

- [_panda_simulator_](https://github.com/justagist/panda_simulator) : Simulation in Gazebo with exposed controllers and
  state feedback using ROS topics and services. The simulated robot uses the same ROS topics and services as the real
  robot when using the [_franka_ros_interface_](https://github.com/justagist/franka_ros_interface).
- [_franka_ros_interface_](https://github.com/justagist/franka_ros_interface) : A ROS API for controlling and managing
  the Franka Emika Panda robot (real and [simulated](https://github.com/justagist/panda_simulator)). Contains
  controllers for the robot (joint position, velocity, torque), interfaces for the gripper, controller manager,
  coordinate frames interface, etc.. Provides almost complete sim-to-real transfer of code.
- [_panda_robot_](https://github.com/justagist/panda_robot) : Python interface providing higher-level control of the
  robot integrated with its gripper, controller manager, coordinate frames manager, etc. It also provides access to the
  kinematics and dynamics of the robot using the [KDL library](http://wiki.ros.org/kdl).

## virtualenv

```console
python3.8 -m pip install virtualenv
python3.8 -m virtualenv venv
source venv/bin/activate
source install.sh
```
