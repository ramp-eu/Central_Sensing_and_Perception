cmake_minimum_required(VERSION 2.4.6)
project(maplistener)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(Boost REQUIRED COMPONENTS signals)
#find_package(OpenMP)
#if(OPENMP_FOUND)
#  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#endif(OPENMP_FOUND)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  roscpp
  tf
  maptogridmap
  mapupdates
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${maptogridmap_INCLUDE_DIRS}
  include
)

add_executable(mapls src/main.cpp)
#target_link_libraries(mapls ${maptogridmap_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mapls ${catkin_LIBRARIES})





