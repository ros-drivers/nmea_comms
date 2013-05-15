cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(nmea_serial_node src/serial_node.cpp src/rx.cpp src/tx.cpp src/checksum.cpp)
rosbuild_link_boost(nmea_serial_node thread)

rosbuild_add_executable(nmea_socket_node src/socket_node.cpp src/rx.cpp src/tx.cpp src/checksum.cpp)
rosbuild_link_boost(nmea_socket_node thread)

rosbuild_add_boost_directories()
