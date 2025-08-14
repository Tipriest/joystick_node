cmake_minimum_required(VERSION 2.8.12)
project(joystick_node)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_STANDARD 17)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
message("!!!CMAKE_CXX_FLAGS is " ${CMAKE_CXX_FLAGS_DEBUG})

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)
message("!!!catkin_INCLUDE_DIRS is  ${catkin_INCLUDE_DIRS}")

# add_compile_options(-mno-avx)

add_executable(joystick_node
    src/main.cpp
    src/joystick.cpp
)
target_link_libraries(joystick_node ${catkin_LIBRARIES})