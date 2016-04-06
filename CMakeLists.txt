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
  OpenCV
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
          ${OpenCV_INCLUDE_DIRS}
)

set(AA_HEADERS
    include/eagle_one_test/PIDController.h
    include/eagle_one_test/ARNavdata.h
    include/eagle_one_test/Odometry.h
    include/eagle_one_test/Drone.h
    include/eagle_one_test/Tag.h
)

add_library(ARNavdata src/ARNavdata.cpp)
target_link_libraries(ARNavdata ${catkin_LIBRARIES})

add_library(Drone src/Drone.cpp)
target_link_libraries(Drone ${catkin_LIBRARIES})
add_dependencies(Drone eagle_one_test_gencpp)

add_library(Odometry src/Odometry.cpp)
target_link_libraries(Odometry ${catkin_LIBRARIES})

add_library(Tag src/Tag.cpp)
target_link_libraries(Tag ${catkin_LIBRARIES})

add_library(PIDController src/PIDController.cpp)
target_link_libraries(PIDController ${catkin_LIBRARIES})
add_dependencies(PIDController eagle_one_test_gencpp)

add_executable(takeoff src/takeoff.cpp)
target_link_libraries(takeoff ${catkin_LIBRARIES})

add_executable(backandforth src/backandforth.cpp)
target_link_libraries(backandforth ${catkin_LIBRARIES})

# add_executable(turnaround src/turnaround.cpp)
# target_link_libraries(turnaround ${catkin_LIBRARIES})

add_executable(forward_takeoff src/forward_takeoff.cpp)
target_link_libraries(forward_takeoff ${catkin_LIBRARIES})

add_executable(sub_nav src/sub_nav.cpp)
target_link_libraries(sub_nav ${catkin_LIBRARIES})

# Link the node and the library
add_executable(tag_dist_test src/tag_dist_test.cpp)
target_link_libraries(tag_dist_test ${catkin_LIBRARIES} Drone ARNavdata)

# Link the node and the library
add_executable(control_test src/control_test.cpp)
target_link_libraries(control_test ${catkin_LIBRARIES} Drone ARNavdata Odometry)

# Link the node and the library
add_executable(print_values src/print_values.cpp)
target_link_libraries(print_values ${catkin_LIBRARIES} Drone ARNavdata Odometry)

# Link the node and the library
add_executable(sub_nav_class src/sub_nav_class.cpp)
target_link_libraries(sub_nav_class ${catkin_LIBRARIES} ARNavdata)

add_executable(takeoff_forward_land src/takeoff_forward_land.cpp)
target_link_libraries(takeoff_forward_land ${catkin_LIBRARIES})

add_executable(takeoff_no_state src/takeoff_no_state.cpp)
target_link_libraries(takeoff_no_state ${catkin_LIBRARIES})

add_executable(pid_client src/pid_client.cpp)
target_link_libraries(pid_client ${catkin_LIBRARIES})
add_dependencies(pid_client eagle_one_test_gencpp)

add_executable(pid_server src/pid_server.cpp)
target_link_libraries(pid_server ${catkin_LIBRARIES})
add_dependencies(pid_server eagle_one_test_gencpp)

add_executable(pid_node src/pid_node.cpp)
target_link_libraries(pid_node ${catkin_LIBRARIES} PIDController)
add_dependencies(pid_node eagle_one_test_gencpp)

add_executable(qc_node src/qc_node.cpp)
target_link_libraries(qc_node ${catkin_LIBRARIES} Drone ARNavdata)
add_dependencies(qc_node eagle_one_test_gencpp)

add_executable(takepicture_mode src/takepicture_mode.cpp)
target_link_libraries(takepicture_mode ${OpenCV_LIBRARIES})
