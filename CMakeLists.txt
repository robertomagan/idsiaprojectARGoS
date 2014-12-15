cmake_minimum_required(VERSION 2.6)
project(argos2_sample)

# Find the ARGoS package
set(CMAKE_MODULE_PATH /usr/share/argos2 ${CMAKE_SOURCE_DIR}/cmake)
find_package(ARGoS REQUIRED)

# Look for the (optional) galib library
find_package(MATLABLIB)
MESSAGE( STATUS "MATLAB LIBRARIES -> " ${MATLABLIB_LIBRARY} )
#if(GALIB_FOUND)
#  include_directories(${GALIB_INCLUDE_DIRS})
#endif(GALIB_FOUND)

# Find the GSL package
find_package(GSL REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIR} ${GSL_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LINK_DIR})

# Set MATLAB include dir
include_directories(/usr/local/MATLAB/R2010b/extern/include/)

# Set MATLAB link dir
#link_directories(/usr/local/MATLAB/R2010b/bin/glnxa64/)

# LCM engine
include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)
lcmtypes_build(CPP_AGGREGATE_HEADER lcmmsgs.hpp)

# Navigation libraries
set(orca_IMPLEMENTATIONS
  navigation/RVO/Agent.cpp
  navigation/RVO/Obstacle.cpp
  navigation/RVO/RVOSimulator.cpp
  navigation/RVO/KdTree.cpp
  )

set(hrvo_IMPLEMENTATIONS
  navigation/HRVO/Agent.cpp
  navigation/HRVO/Goal.cpp
  navigation/HRVO/HRVOSimulator.cpp
  navigation/HRVO/KdTree.cpp
  navigation/HRVO/Obstacle.cpp
  navigation/HRVO/RoadmapVertex.cpp
  navigation/HRVO/Vector2.cpp
  )
  
set(agentvo_IMPLEMENTATIONS
  navigation/Agent.cpp
  navigation/HLAgent.cpp
  navigation/HRVOAgent.cpp
  navigation/ORCAAgent.cpp
  )

# Include the simulation compile flags
include(${CMAKE_SOURCE_DIR}/cmake/SimulationConfig.cmake)

# Descend into the controllers directory
add_subdirectory(controllers)

add_subdirectory(includes)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)

# Descend into the embedding directory
add_subdirectory(embedding)
