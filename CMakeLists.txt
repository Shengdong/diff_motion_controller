cmake_minimum_required (VERSION 2.6)
project(usb_can_node)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}
                    /usr/local/include
                    )
find_package(catkin REQUIRED COMPONENTS
             apriltags_ros
             cmake_modules
             roscpp
             geometry_msgs
             message_generation
             std_msgs
)

add_message_files(DIRECTORY msg
  FILES
  AbsolutePose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

set(CMAKE_USE_WIN32_THREADS_INIT TRUE)
set(CMAKE_USE_PTHREADS_INIT TRUE)
set(CMAKE_USE_PROC_INIT TRUE)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCAN REQUIRED)
find_package(Boost REQUIRED)

add_definitions("-std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usb_can_test
  CATKIN_DEPENDS apriltags_ros roscpp geometry_msgs std_msgs message_runtime
  DEPENDS Eigen3 BOOST
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_library(usb_can
            src/dualmotor.cpp
            src/device.cpp
            src/usb_can.cpp
)

target_link_libraries(usb_can
  ${catkin_LIBRARIES}
  ${PCAN_LIBS}
  ${Eigen3_LIBRARIES}
)

add_dependencies(usb_can usb_can_node_generate_messages_cpp)

add_executable(usb_can_node src/usb_can_node.cpp)
target_link_libraries(usb_can_node 
                      ${catkin_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT} 
                      ${PCAN_LIBS}
                      usb_can
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS usb_can usb_can_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/usb_can_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


