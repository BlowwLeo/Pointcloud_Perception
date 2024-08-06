cmake_minimum_required(VERSION 3.14)
project(autoware_point_types)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  ../
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)



ament_auto_package()
