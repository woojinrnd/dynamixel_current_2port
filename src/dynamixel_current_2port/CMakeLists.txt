cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_current_2port)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(dynamixel_current_2port src/dynamixel_current_2port.cpp ${src/dynamixel_current_2port.hpp})

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamixel_sdk
  dynamixel_workbench_msgs
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(RBDL REQUIRED)




catkin_package(
  INCLUDE_DIRS include
    LIBRARIES dynamixel_current_2port
  CATKIN_DEPENDS
    cmake_modules 
    dynamixel_sdk 
    dynamixel_workbench_msgs 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS 
    system_lib
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIR}
  ${dynamixel_workbench_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ./include
)

add_executable(
  dynamixel_current_2port
  src/main.cpp
  src/dynamixel_current_2port.cpp ${dynamixel_current_2port.hpp}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_current_2port ${catkin_LIBRARIES})

install(TARGETS dynamixel_current_2port
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


