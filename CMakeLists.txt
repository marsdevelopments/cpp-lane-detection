cmake_minimum_required(VERSION 3.2)

project(cpp-lane-detection)

find_package( OpenCV REQUIRED )
include_directories((src/include))

# include_directories(source)

add_executable(cpp-lane-detection
  main.cpp
  src/LanePoints.cpp
  src/PreProcessing.cpp
  src/LaneTracking.cpp
  src/TrackerManager.cpp
  src/RandomLaneBuilder.cpp
)

target_link_libraries( cpp-lane-detection ${OpenCV_LIBS} )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")