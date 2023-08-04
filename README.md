# BUILD
mkdir -p [what you want folder name]/[src]  
cd [what you want folder name]/[src]  
git clone https://github.com/woojinrnd/dynamixel_current_2port.git  
cd ..  
catkin_make  

# RUN
source devel/setup.bash  
split 3 terminal  
[terminal 1] roscore  
[terminal 2] rosrun dynamixel_current_2port Move_decision_node  
[terminal 3] rosrun dynamixel_current_2port dynamixel_current_2port  


# ROS GRPAH
![0730(2)_rosgraph](https://github.com/woojinrnd/dynamixel_current_2port/assets/122770475/a4cfefc6-b8cb-490b-95da-91e7643174ce)

# Souce Tree
```
src
├── 0515_rosgraph.png
├── 0607_rosgraph2.png
├── 0607_rosgraph.png
├── 0621_rosgraph.png
├── 0721_rosgraph.png
├── 0730(2)_rosgraph.png
├── 0730_rosgraph.png
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── dynamixel_current_2port
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── callback.hpp
│   │   ├── dynamixel_controller.hpp
│   │   ├── dynamixel.hpp
│   │   ├── img_proc.hpp
│   │   ├── Move_decision.hpp
│   │   ├── sensor.hpp
│   │   └── Walkingpattern_generator.hpp
│   ├── launch
│   │   └── dynamixel_current_2port.launch
│   ├── package.xml
│   ├── src
│   │   ├── callback.cpp
│   │   ├── dynamixel_controller.cpp
│   │   ├── dynamixel.cpp
│   │   ├── fsr_ux_420_short.ino
│   │   ├── img_proc.cpp
│   │   ├── main_2.cpp
│   │   ├── main.cpp
│   │   ├── Move_decision.cpp
│   │   ├── Move_decision_node.cpp
│   │   ├── sensor.cpp
│   │   └── Walkingpattern_generator.cc
│   └── srv
│       ├── RL_NeckAngle.srv
│       ├── Select_Motion.srv
│       ├── Turn_Angle.srv
│       └── UD_NeckAngle.srv
└── README.md
```
