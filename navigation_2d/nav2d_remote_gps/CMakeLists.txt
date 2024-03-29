cmake_minimum_required(VERSION 2.8.3)
project(nav2d_remote_gps)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav2d_navigator
  nav2d_operator
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    nav2d_navigator
    nav2d_operator
    roscpp
    sensor_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(remote_gps src/remote_gps.cpp)
add_executable(remote_gps_test src/remote_gps_test.cpp)
add_executable(robot_tf_broadcast src/robot_tf_broadcast.cpp)
add_executable(start_mark_tf_broadcast src/start_mark_tf_broadcast.cpp)

add_dependencies(remote_gps ${catkin_EXPORTED_TARGETS})
add_dependencies(remote_gps_test ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_tf_broadcast ${catkin_EXPORTED_TARGETS})
add_dependencies(start_mark_tf_broadcast ${catkin_EXPORTED_TARGETS})

target_link_libraries(remote_gps ${catkin_LIBRARIES})
target_link_libraries(remote_gps_test ${catkin_LIBRARIES})
target_link_libraries(robot_tf_broadcast ${catkin_LIBRARIES})
target_link_libraries(start_mark_tf_broadcast ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS remote_gps
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_remote_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
