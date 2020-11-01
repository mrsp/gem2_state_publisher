cmake_minimum_required(VERSION 2.8.3)
project(gem2_state_publisher)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
SET(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -DEIGEN_NO_DEBUG -march=native -Wl,--no-as-needed") 
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  message_filters
)
find_package(Boost REQUIRED COMPONENTS system)


find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO pinocchio REQUIRED)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gem2_state_publisher
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs  message_filters
  DEPENDS Eigen3 pinocchio 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(  
${PINOCCHIO_LIBRARY_DIRS})

add_executable(gem2_state_publisher src/gem2_state_publisher_driver.cpp src/gem2_state_publisher.cpp 
src/butterworthHPF.cpp src/butterworthLPF.cpp src/MovingAverageFilter.cpp src/JointDF.cpp src/differentiator.cpp)
target_link_libraries(gem2_state_publisher ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PINOCCHIO_LIBRARIES}  ${Boost_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

add_dependencies(gem2_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
