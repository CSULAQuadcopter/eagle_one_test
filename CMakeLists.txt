cmake_minimum_required(VERSION 2.8.3)
project(eagle_one_test)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  ardrone_tutorials
  roscpp
  rospy
)

catkin_package()


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(takeoff src/takeoff.cpp)
add_executable(backandforth src/backandforth.cpp)
# add_executable(turnaround src/turnaround.cpp)
add_executable(forward_takeoff src/forward_takeoff.cpp)
add_executable(sub_nav src/sub_nav.cpp)
add_executable(takeoff_forward_land src/takeoff_forward_land.cpp)
add_executable(takeoff_no_state src/takeoff_no_state.cpp)

target_link_libraries(takeoff ${catkin_LIBRARIES})
target_link_libraries(backandforth ${catkin_LIBRARIES})
# target_link_libraries(turnaround ${catkin_LIBRARIES})
target_link_libraries(forward_takeoff ${catkin_LIBRARIES})
target_link_libraries(sub_nav ${catkin_LIBRARIES})
target_link_libraries(takeoff_forward_land ${catkin_LIBRARIES})
target_link_libraries(takeoff_no_state ${catkin_LIBRARIES})
