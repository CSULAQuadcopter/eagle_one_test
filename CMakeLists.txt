cmake_minimum_required(VERSION 2.8.3)
project(eagle_one_test)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  ardrone_tutorials
  roscpp
  rospy
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

set(AA_HEADERS
    include/eagle_one_test/ARNavdata.h
    include/eagle_one_test/Drone.h
)

add_library(ARNavdata src/ARNavdata.cpp)
target_link_libraries(ARNavdata ${catkin_LIBRARIES})

add_library(Drone src/Drone.cpp)
target_link_libraries(Drone ${catkin_LIBRARIES})

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
target_link_libraries(control_test ${catkin_LIBRARIES} Drone ARNavdata)

# Link the node and the library
add_executable(sub_nav_class src/sub_nav_class.cpp)
target_link_libraries(sub_nav_class ${catkin_LIBRARIES} ARNavdata)

add_executable(takeoff_forward_land src/takeoff_forward_land.cpp)
target_link_libraries(takeoff_forward_land ${catkin_LIBRARIES})

add_executable(takeoff_no_state src/takeoff_no_state.cpp)
target_link_libraries(takeoff_no_state ${catkin_LIBRARIES})
