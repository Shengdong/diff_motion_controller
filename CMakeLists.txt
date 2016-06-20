cmake_minimum_required (VERSION 2.6)
project(motion_controller)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}
                    /usr/local/include
                    )

find_package(catkin REQUIRED COMPONENTS
             apriltags_ros
             cmake_modules
             roscpp
             geometry_msgs
             std_msgs
)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCAN REQUIRED)
find_package(Boost REQUIRED)

add_definitions("-std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES usb_can_test
  CATKIN_DEPENDS apriltags_ros roscpp geometry_msgs std_msgs message_runtime
  DEPENDS Eigen3 BOOST
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(motion_controller
            src/dualmotor.cpp
            src/singlemotor.cpp
            src/usb_can_device.cpp
            src/motion_controller.cpp
)

target_link_libraries(motion_controller
  ${catkin_LIBRARIES}
  ${PCAN_LIBS}
  ${Eigen3_LIBRARIES}
)

add_executable(motion_controller_node src/motion_controller_node.cpp)

target_link_libraries(motion_controller_node
                      ${catkin_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT} 
                      ${PCAN_LIBS}
                      motion_controller
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS motion_controller motion_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/motion_controller_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


