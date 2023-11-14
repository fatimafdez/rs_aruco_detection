cmake_minimum_required(VERSION 3.0.2)
project(rs_aruco_detection)
# add_compile_options(-std=c++11)

# --- Dependencies ---

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


# --- catkin specific configuration ---

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rs_aruco_detection
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
#  DEPENDS system_lib
)

# --- Build ---

SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

IF(DEFINED CMAKE_BUILD_TYPE)
    SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
ENDIF()

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

IF(NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND
    NOT ${CMAKE_BUILD_TYPE} STREQUAL "Release")
    MESSAGE(FATAL_ERROR "Only Release and Debug build types are allowed.")
ENDIF()

# --- include directories ---
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# --- Link directories ---
link_directories(
  include
  ${catkin_LIBRARY_DIRS}
)

# --- Build ---
file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_SRCS "src/*.c*")
file(GLOB_RECURSE SRC_HDRS "src/*.h*")

add_library(${PROJECT_NAME} ${SRC_SRCS} ${SRC_HDRS} ${SRC_INCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_node utils/rs_aruco_position.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node2 src/aruco_detector.cpp)
target_link_libraries(${PROJECT_NAME}_node2 ${PROJECT_NAME})

#add_executable(rs_aruco_detector src/aruco_detector.cpp)
#target_link_libraries(rs_aruco_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(rs_aruco_detector ${catkin_EXPORTED_TARGETS})

#add_executable(rs_aruco_position src/ArucoPosition.cpp)
#target_link_libraries(rs_aruco_position ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(rs_aruco_position ${catkin_EXPORTED_TARGETS})






## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rs_aruco_detection.cpp
# )

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)