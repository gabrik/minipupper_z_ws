cmake_minimum_required(VERSION 2.8.3)
project(scan_relay)

## Compile as C++11, supported in ROS Kinetic and newer
set (IS_WARLL "OFF") # "ON" or "OFF"
if(${IS_WARLL} MATCHES "OFF")
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  message(STATUS "optional:-std=c++11")
else()
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
  message(STATUS "optional:-std=c++11 -Wall -Wextra ")
endif()
set(CMAKE_C_STANDARD 11)


# set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
# set (CMAKE_VERBOSE_MAKEFILE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ldlidar
# CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/core/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/dataprocess/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/filter/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/logger/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/networkcom/
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/serialcom/
  ${catkin_INCLUDE_DIRS}
)

file(GLOB  MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

find_library(ZENOH zenohc)
set(EXTRA_LIBS ${ZENOH} pthread)

add_executable(${PROJECT_NAME}_node
  ${MAIN_SRC}
)



target_link_libraries(${PROJECT_NAME}_node ${EXTRA_LIBS} ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ldlidar.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
