cmake_minimum_required(VERSION 2.8.3)
project(kdl_manager_tutorial)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  generic_control_toolbox
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp generic_control_toolbox
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(load_chain src/load_chain.cpp)
target_link_libraries(load_chain ${catkin_LIBRARIES})
add_dependencies(load_chain ${catkin_EXPORTED_TARGETS})
