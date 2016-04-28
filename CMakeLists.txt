cmake_minimum_required (VERSION 2.6)
project(usb_can_test_node)

set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-gcc")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(${PROJECT_SOURCE_DIR}
                    /usr/local/include
                    )
find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             roscpp
)

set(CMAKE_USE_WIN32_THREADS_INIT TRUE)
set(CMAKE_USE_PTHREADS_INIT TRUE)
set(CMAKE_USE_PROC_INIT TRUE)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

find_package(Threads REQUIRED)

IF(CMAKE_USE_PTHREADS_INIT)
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")
ENDIF(CMAKE_USE_PTHREADS_INIT)

find_package(PCAN REQUIRED)
find_package(Boost REQUIRED)

add_definitions("-std=c++11 -Wall")

catkin_package(
  LIBRARIES usb_can_test
  CATKIN_DEPENDS roscpp
  DEPENDS BOOST
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(usb_can_test_node src/USB_CAN_Test.cpp)
target_link_libraries(usb_can_test_node ${CMAKE_THREAD_LIBS_INIT} ${PCAN_LIBS})

## Mark executables and/or libraries for installation
install(TARGETS usb_can_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/usb_can_test_node
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


