cmake_minimum_required(VERSION 2.8.3)
project(xld_simulator)

add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  uav_utils
)


catkin_package()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(myModel src/dynamics/myModel.cpp)

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
add_executable(xld_simulator_node
  src/xld_simulator_node.cpp)

target_link_libraries(xld_simulator_node
   ${catkin_LIBRARIES}
   myModel
)
