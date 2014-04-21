cmake_minimum_required(VERSION 2.8.3)
project(nmea_comms)

find_package(catkin REQUIRED COMPONENTS roscpp nmea_msgs rostest)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS src/rx.cpp src/tx.cpp src/checksum.cpp)

add_executable(nmea_serial_node src/serial_node.cpp ${${PROJECT_NAME}_SRCS})
set_target_properties(nmea_serial_node PROPERTIES OUTPUT_NAME serial_node PREFIX "")
add_dependencies(nmea_serial_node nmea_msgs_gencpp)
target_link_libraries(nmea_serial_node ${catkin_LIBRARIES})

add_executable(nmea_socket_node src/socket_node.cpp ${${PROJECT_NAME}_SRCS})
set_target_properties(nmea_socket_node PROPERTIES OUTPUT_NAME socket_node PREFIX "")
add_dependencies(nmea_socket_node nmea_msgs_gencpp)
target_link_libraries(nmea_socket_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS nmea_serial_node nmea_socket_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  add_rostest_gtest(nmea_socket_node_test test/socket_node.test test/test_socket_node.cpp)
  target_link_libraries(nmea_socket_node_test ${catkin_LIBRARIES})
endif()
