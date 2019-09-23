cmake_minimum_required(VERSION 2.8.3)
project(navi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  genmsg
  message_generation
)

add_message_files(
  FILES
  rio_to_pc.msg
)

find_package(OpenCV REQUIRED)

generate_messages(DEPENDENCIES std_msgs)

set( sources
  src/main.cpp
  src/c_ekf.cpp
  src/CoordinateConv.cpp
)

set( headers
  src/main.h
  src/common_def.h
  src/c_ekf.h
  src/CoordinateConv.h
)

catkin_package(
  LIBRARIES navigation
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node ${headers} ${sources})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

