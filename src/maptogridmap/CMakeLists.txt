cmake_minimum_required(VERSION 2.4.6)
project(maptogridmap)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  roscpp
  tf
  laser_geometry
  mapupdates
)
add_message_files(
  FILES
  Nodes.msg
  Edges.msg
  Annotations.msg
  Annotation.msg
  Vertex.msg
  Edge.msg
  Graph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(map2gm src/main.cpp src/gmap.cpp)
target_link_libraries(map2gm ${catkin_LIBRARIES})
add_dependencies(map2gm maptogridmap_generate_messages_cpp)





