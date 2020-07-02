cmake_minimum_required(VERSION 2.8.3)
project(wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ssoni_mobile_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wheel_odometry
  CATKIN_DEPENDS roscpp std_msgs ssoni_mobile_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_publisher src/wheel_odometry.cpp)
target_link_libraries(odometry_publisher ${catkin_LIBRARIES})
add_dependencies(odometry_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(odometry_publisher wheel_odometry_generate_messages_cpp)
