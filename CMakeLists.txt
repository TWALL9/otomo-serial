cmake_minimum_required(VERSION 3.0.2)
project(otomo-serial)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  otomo_msgs
  roslaunch
)

find_package(async_comm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable(${PROJECT_NAME}_node 
  src/serial_node.cpp
  src/kiss_encode.cpp
  src/kiss_decode.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${async_comm_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
