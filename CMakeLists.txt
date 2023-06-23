cmake_minimum_required(VERSION 3.2)

project(cpp-lane-detection)

find_package( OpenCV REQUIRED )
include_directories((src/include))

# include_directories(source)

add_executable(cpp-lane-detection
  main.cpp
  src/pre_processing.cpp
  src/lane_detection.cpp
  src/random_lane_detection.cpp
  src/random_lane_tracking.cpp
  src/tracker_manager.cpp
)

target_link_libraries( cpp-lane-detection ${OpenCV_LIBS} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")