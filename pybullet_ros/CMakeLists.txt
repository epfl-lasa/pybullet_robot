cmake_minimum_required(VERSION 3.0.2)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
    franka_description
    )

catkin_python_setup()

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    nodes/torque_control_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

