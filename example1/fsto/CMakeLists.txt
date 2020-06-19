cmake_minimum_required(VERSION 2.8.3)

project(example1)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_filters
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  image_transport
  cv_bridge
  quadrotor_msgs
  visualization_msgs
)

include_directories(
    ${OMPL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
    ../../include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME}_node src/fsto_node.cpp src/fsto.cpp src/glbmap.cpp src/r3planner.cpp src/traj_gen.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
