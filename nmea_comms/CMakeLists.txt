cmake_minimum_required(VERSION 2.8.3)
project(nmea_comms)

find_package(catkin REQUIRED COMPONENTS nmea_msgs roscpp)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(nmea_serial_node src/serial.cpp)
target_link_libraries(nmea_serial_node ${catkin_LIBRARIES})
add_dependencies(nmea_serial_node nmea_msgs_gencpp)
