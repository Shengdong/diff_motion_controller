set(PCAN_INCLUDE
    ${PROJECT_SOURCE_DIR}
)
set(PCAN_LIBS
    ${PROJECT_SOURCE_DIR}/lib/libGinkgo_Driver.so
    ${PROJECT_SOURCE_DIR}/lib/libusb.so
)

# handle the QUIETLY and REQUIRED arguments and set PYLON_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PCAN REQUIRED_VARS PCAN_INCLUDE PCAN_LIBS)

if(PCAN_FOUND)
  set( PCAN_INCLUDE_DIRS ${PCAN_INCLUDE} )
  set( PCAN_LIBRARIES ${PCAN_LIBS} )
endif()

mark_as_advanced(PCAN_INCLUDE PCAN_LIBS)
