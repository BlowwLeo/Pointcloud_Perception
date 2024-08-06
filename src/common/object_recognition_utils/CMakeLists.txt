cmake_minimum_required(VERSION 3.14)
project(object_recognition_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(object_recognition_utils SHARED
  src/predicted_path_utils.cpp
  src/conversion.cpp
)


ament_auto_package()
