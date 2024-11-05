
cmake_minimum_required(VERSION 3.14.0)
project(robot_control_solution)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")

# Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  ur_kinematics
)
find_package(ruckig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)  # Add Pybind11 for the Python module

# Define catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_control_library
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ur_kinematics_INCLUDE_DIRS}
  include
  ${ruckig_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pybind11_INCLUDE_DIRS}  # Include Pybind11 directories
)

add_definitions(
  -DIKFAST_HAS_LIBRARY
  -DIKFAST_NO_MAIN
  -DUR5_PARAMS
)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Main robot control library
add_library(robot_control_library SHARED src/robot_control.cpp src/robot_kinematics.cpp)
target_link_libraries(robot_control_library ${catkin_LIBRARIES} ruckig::ruckig Eigen3::Eigen)
set_target_properties(robot_control_library PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Executables
add_executable(sin_control_node src/sin_control.cpp)
add_executable(robot_motion_control_node src/robot_motion_control.cpp)

target_link_libraries(sin_control_node ${catkin_LIBRARIES})
target_link_libraries(robot_motion_control_node robot_control_library Eigen3::Eigen)

# Python module using Pybind11
pybind11_add_module(robot_control_py src/robot_control_py.cpp)  # Create the Python module
target_link_libraries(robot_control_py PRIVATE robot_control_library ${catkin_LIBRARIES} ruckig::ruckig Eigen3::Eigen)
set_target_properties(robot_control_py PROPERTIES
    OUTPUT_NAME "robot_kinematics"  # Name of the module when imported in Python
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}  # Install to catkin Python destination
)

# Install Python module to appropriate directory
install(TARGETS robot_control_py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)