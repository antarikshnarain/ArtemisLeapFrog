cmake_minimum_required(VERSION 2.8.12)
project(crater_landing)

find_package(gazebo REQUIRED)
find_package(ignition-transport4 REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(UpdateRedZoneScore SHARED UpdateRedZoneScore.cpp)
target_link_libraries(UpdateRedZoneScore
  ${GAZEBO_LIBRARIES}
  ignition-transport4::ignition-transport4
)

add_library(UpdateBlueZoneScore SHARED UpdateBlueZoneScore.cpp)
target_link_libraries(UpdateBlueZoneScore
  ${GAZEBO_LIBRARIES}
  ignition-transport4::ignition-transport4
)

add_library(UpdateGreenZoneScore SHARED UpdateGreenZoneScore.cpp)
target_link_libraries(UpdateGreenZoneScore
  ${GAZEBO_LIBRARIES}
  ignition-transport4::ignition-transport4
)
