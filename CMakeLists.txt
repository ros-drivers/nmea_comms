cmake_minimum_required(VERSION 2.8.3)
project(nmea_comms)

find_package(catkin REQUIRED COMPONENTS roscpp nmea_msgs)

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

##########
## Docs ##
##########

add_custom_target (${PROJECT_NAME}_doc 
  rosdoc_lite -o ${CATKIN_DEVEL_PREFIX}/doc/${PROJECT_NAME} ${CATKIN_DEVEL_PREFIX}/../src/nmea/${PROJECT_NAME}
)

#############
## Testing ##
#############

FILE(GLOB LINT_SRCS src/*.cpp include/${PROJECT_NAME}/*.h)
add_custom_target(${PROJECT_NAME}_cpplint
  COMMAND cpplint --filter=-whitespace/line_length ${LINT_SRCS})
