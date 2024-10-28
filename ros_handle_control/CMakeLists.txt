cmake_minimum_required(VERSION 3.0.2)
project(ros_handle_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  turtlesim
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_turtle src/teleop_turtle.cpp)
add_executable(draw_chessboard src/draw_chessboard.cpp)

target_link_libraries(teleop_turtle ${catkin_LIBRARIES})
target_link_libraries(draw_chessboard ${catkin_LIBRARIES})
