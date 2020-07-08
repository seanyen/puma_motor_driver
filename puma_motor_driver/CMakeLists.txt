cmake_minimum_required(VERSION 2.8.3)
project(puma_motor_driver)

if (WIN32)
set(TARGET_ARCH "Win32")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_ARCH "x64")
endif()

# Create locations to place the PCAN-Basic library
file(MAKE_DIRECTORY
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)

# Pull down the PCAN-Basic library
include(ExternalProject)
ExternalProject_Add(
    pcan-basic
    URL https://www.peak-system.com/fileadmin/media/files/pcan-basic.zip
    URL_MD5 d388e723046e7b2f6bd06489a12a3d96
    PREFIX ${CMAKE_BINARY_DIR}/pcan-basic
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
                    <SOURCE_DIR>/include ${CMAKE_BINARY_DIR}/installed/pcan-basic/include
    COMMAND ${CMAKE_COMMAND} -E copy
            <SOURCE_DIR>/${TARGET_ARCH}/PCANBasic.dll ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin
    COMMAND ${CMAKE_COMMAND} -E copy
            <SOURCE_DIR>/${TARGET_ARCH}/VC_LIB/PCANBasic.lib ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib
    LOG_DOWNLOAD ON
    LOG_INSTALL ON
    )

# Initialize include paths and library paths
set(pcan_LIBRARIES ${CMAKE_BINARY_DIR}/installed/pcan-basic/lib/PCANBasic.lib)
set(pcan_INCLUDE_DIRS ${CMAKE_BINARY_DIR}/installed/pcan-basic/include)
else()
set(pcan_LIBRARIES)
set(pcan_INCLUDE_DIRS)
endif()

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  puma_motor_msgs
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES puma_motor_driver
  CATKIN_DEPENDS diagnostic_updater puma_motor_msgs roscpp sensor_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pcan_INCLUDE_DIRS}  
)

if (WIN32)
  file(GLOB CAN_CPP_FILES src/peakcan_gateway.cpp)
else()
  file(GLOB CAN_CPP_FILES src/socketcan_gateway.cpp)
endif()

add_library(puma_motor_driver
  ${CAN_CPP_FILES}
  src/driver.cpp
  src/serial_gateway.cpp
  src/multi_driver_node.cpp
  src/diagnostic_updater.cpp
)

if (WIN32)
add_dependencies(puma_motor_driver pcan-basic)
endif()

target_link_libraries(puma_motor_driver
  ${catkin_LIBRARIES}
  ${pcan_LIBRARIES}
)
add_dependencies(puma_motor_driver puma_motor_msgs_gencpp)

add_executable(puma_multi_controller_node src/multi_controller_node.cpp)

target_link_libraries(puma_multi_controller_node
  puma_motor_driver
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS puma_motor_driver puma_multi_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (WIN32)
# Remember to install the DLL side-by-side to ROS application
install(FILES
    ${CMAKE_BINARY_DIR}/installed/pcan-basic/bin/PCANBasic.dll
    DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  file(GLOB LINT_SRCS
    src/*.cpp
    include/puma_motor_driver/driver.h
    include/puma_motor_driver/gateway.h
    include/puma_motor_driver/message.h
    include/puma_motor_driver/serial_gateway.h
    include/puma_motor_driver/socketcan_gateway.h
    include/puma_motor_driver/multi_driver_node.h
    include/puma_motor_driver/diagnostic_updater.h
    test/*.cpp)
  roslint_cpp(${LINT_SRCS})
  roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/mecanum.launch)
  roslaunch_add_file_check(launch/omni.launch)

  catkin_add_gtest(fixed_point_test test/fixed_point_test.cpp)
endif()
