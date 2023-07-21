# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/local/include".split(';') if "${prefix}/include;/usr/local/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cmake_modules;dynamixel_sdk;dynamixel_workbench_msgs;roscpp;std_msgs;sensor_msgs;geometry_msgs;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldynamixel_current_2port;/usr/local/lib/libboost_system.so.1.81.0".split(';') if "-ldynamixel_current_2port;/usr/local/lib/libboost_system.so.1.81.0" != "" else []
PROJECT_NAME = "dynamixel_current_2port"
PROJECT_SPACE_DIR = "/home/woojin/dynamixel_current/dynamixel_current_2port/install"
PROJECT_VERSION = "0.0.0"
