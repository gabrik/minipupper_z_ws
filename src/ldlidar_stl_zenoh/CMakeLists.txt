cmake_minimum_required(VERSION 2.8.3)
project(ldlidar_stl_zenoh)

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
    sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ldlidar
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


# fetch latest argparse
include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)



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

file(GLOB  LDLIDAR_DRI_CORE ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/core/*.cpp)
file(GLOB  LDLIDAR_DRI_DATARPC ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/dataprocess/*.cpp)
file(GLOB  LDLIDAR_DRI_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/filter/*.cpp)
file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/logger/*.cpp)
file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/networkcom/*.cpp)
file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/serialcom/*.cpp)

file(GLOB  MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)


# Zenoh Pico
# set(BUILD_EXAMPLES OFF CACHE BOOL "Disables examples" FORCE)
# set(BUILD_TESTING OFF CACHE BOOL "Disables tests" FORCE)
# set(BUILD_MULTICAST OFF CACHE BOOL "Disables multicast tests" FORCE)
# set(BUILT_INTEGRATION OFF CACHE BOOL "Disables integration tests " FORCE)
# set(BUILD_TOOLS OFF CACHE BOOL "Disables build tools" FORCE)

# include_directories(${CMAKE_SOURCE_DIR}/zenoh-pico/include)
# set(Z_BUILD_FLAGS "-DZENOH_LINUX=1 -DZENOH_C_STANDARD=99 -DZ_LINK_TCP=1 -DZ_LINK_UDP_MULTICAST=1 -DZ_LINK_UDP_UNICAST=1 -DZ_SCOUTING_UDP=1 -DZ_MULTI_THREAD=1")
# set(EXTRA_LIBS zenohpico -pthread argparse)

##

# include(FetchContent)
# FetchContent_Declare(
#     zenoh
#     GIT_REPOSITORY https://github.com/gabrik/zenoh-c
#     GIT_TAG build-flags

# )
# FetchContent_MakeAvailable(zenoh)
find_library(ZENOH zenohc)

# include_directories(${zenoh_SOURCE_DIR}/include)
set(EXTRA_LIBS ${ZENOH} -pthread argparse)
set(CMAKE_C_STANDARD 11)

add_executable(${PROJECT_NAME}_node
  ${MAIN_SRC}
  ${LDLIDAR_DRI_CORE}
  ${LDLIDAR_DRI_DATARPC}
  ${LDLIDAR_DRI_FILTER}
  ${LDLIDAR_DRI_LOGGER}
  ${LDLIDAR_DRI_NETWORK}
  ${LDLIDAR_DRI_SERIAL}
)

# target_link_libraries(${PROJECT_NAME}_node pthread ${catkin_LIBRARIES})

## Adding target link properties quadruped_controller
# set_target_properties(${PROJECT_NAME}_node PROPERTIES COMPILE_FLAGS ${Z_BUILD_FLAGS} )
target_link_libraries(${PROJECT_NAME}_node ${EXTRA_LIBS} ${catkin_LIBRARIES})
