cmake_minimum_required(VERSION 2.8.3)
project(hku_m100_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dji_sdk
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(hku_m100_pcsim_gazebo_bridge src/hku_m100_pcsim_gazebo_bridge.cpp)

target_link_libraries(hku_m100_pcsim_gazebo_bridge
  ${catkin_LIBRARIES}
)
