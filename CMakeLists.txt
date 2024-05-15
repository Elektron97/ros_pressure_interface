cmake_minimum_required(VERSION 3.0.2)
project(ros_pressure_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include

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

add_executable(test_pressure  
  src/ros_pressure_interface/test_pressure.cpp
)

target_link_libraries(test_pressure 
  ${catkin_LIBRARIES} 
  ${ROS_LIBRARIES} ${CASADI_LIB})

add_dependencies(test_pressure
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})




