cmake_minimum_required(VERSION 3.0.2)
project(ros_pressure_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

catkin_install_python(
  PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)