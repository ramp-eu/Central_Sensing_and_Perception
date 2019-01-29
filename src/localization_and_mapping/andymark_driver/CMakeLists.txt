cmake_minimum_required(VERSION 2.8.3)
project(andymark_driver)

# Build ROS-independent library.
find_package(console_bridge REQUIRED  )
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(holonomic src/holonomic.cpp)
target_link_libraries(holonomic ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(holonomic_node src/holonomic.cpp)
target_link_libraries(holonomic_node holonomic ${catkin_LIBRARIES})


#install(TARGETS LMS1xx LMS1xx_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(PROGRAMS scripts/find_sick scripts/set_sick_ip
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
