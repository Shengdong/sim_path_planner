cmake_minimum_required(VERSION 2.8.3)
project(sim_lattice_planner_2d)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  actionlib_msgs
  actionlib
)

add_action_files(
  DIRECTORY action
  FILES Goal.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  LIBRARIES sim_lattice_planner_2d
  DEPENDS boost eigen actionlib_msgs actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(sim_lattice_planner_2d

  src/GraphSearch.cpp
  src/Heuristics.cpp
  src/LatticePlanner.cpp
  src/LatticePose.cpp
  src/OrientationLUT.cpp
  src/Pose.cpp
  src/PrimitivePath.cpp
  src/State.cpp
  src/StateGrid.cpp
  src/StateLattice.cpp
)

target_link_libraries(sim_lattice_planner_2d
  ${catkin_LIBRARIES}
)

add_executable(sim_lattice_planner_2d_node
  src/lattice_planner_node.cpp
)

target_link_libraries(sim_lattice_planner_2d_node
  ${catkin_LIBRARIES}
  sim_lattice_planner_2d
)

target_link_libraries(sim_lattice_planner_2d
  ${catkin_LIBRARIES}
)
