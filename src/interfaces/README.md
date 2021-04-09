# PyBullet Simulation Interfaces

This is the place where different interfaces are implemented for the communication between the PyBullet simulation and
controllers. There could also be a ROS interface, if needed.

## Franka ZMQ Simulation Interface

This is the equivalent to the Franka LWI Communication Protocol
from [here](https://github.com/epfl-lasa/franka_lightweight_interface/blob/main/include/franka_lightweight_interface/franka_lwi_communication_protocol.h)
, adapted for the PyBullet simulation, i.e. taking the state from the simulation and sending it as a ZMQ publisher as
well as receiving commands as a ZMQ subscriber.