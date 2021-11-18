cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_obstacle_avoidence)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
