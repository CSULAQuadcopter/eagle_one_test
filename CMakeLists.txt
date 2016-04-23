cmake_minimum_required(VERSION 2.8.3)
project(eagle_one_test)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  ardrone_tutorials
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
  dynamic_reconfigure
)

add_message_files(
    FILES
    Test.msg
    Pid_send.msg
    Pid_receive.msg
)

add_service_files(
    FILES
    Pid2.srv
    State.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

