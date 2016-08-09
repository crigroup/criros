cmake_minimum_required(VERSION 2.8.3)
project(criros)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

find_package(openrave REQUIRED)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
