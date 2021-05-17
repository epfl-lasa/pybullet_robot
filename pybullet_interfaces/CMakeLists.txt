cmake_minimum_required(VERSION 3.0.2)
project(pybullet_interfaces)

find_package(catkin REQUIRED COMPONENTS
    rospy
    )

catkin_python_setup()

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    nodes/simulation_node
    nodes/torque_control_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

