cmake_minimum_required(VERSION 2.8.3)
project(ball_detection)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  message_generation
  visualization_msgs
  dynamic_reconfigure
  tf
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )


generate_dynamic_reconfigure_options(
  cfg/BallDetection.cfg
)


catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  image_transport
#  visualization_msgs
#  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(ball_detect src/ball_detect.cpp)
add_dependencies(ball_detect core_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(ball_detect
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

#add_executable(ball_detect_color_only src/ball_detect_color_only.cpp)
#add_dependencies(ball_detect_color_only core_msgs_generate_messages_cpp)
#target_link_libraries(ball_detect_color_only
#  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
#)

#add_executable(tf_listener src/tf_listener.cpp)
#target_link_libraries(tf_listener ${OpenCV_LIBS} ${cv_bridge_LIBRARIES} ${catkin_LIBRARIES})
