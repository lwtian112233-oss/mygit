cmake_minimum_required(VERSION 3.5)
project(lane_detection LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lane_detector
  src/main.cpp
  src/lane_detector.cpp
)

target_link_libraries(lane_detector ${OpenCV_LIBS})

add_executable(headless_runner
  src/headless_runner.cpp
  src/lane_detector.cpp
)

target_link_libraries(headless_runner ${OpenCV_LIBS})
