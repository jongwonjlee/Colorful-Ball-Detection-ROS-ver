cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)
find_package(catkin REQUIRED

  message_generation
  std_msgs
)

add_message_files( DIRECTORY msg
  FILES
  ball_position.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
