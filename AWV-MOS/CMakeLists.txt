cmake_minimum_required(VERSION 2.8.3)
project(awv_mos)

option(USE_EVALUATION_POINT_TYPE "Use evaluation point type" OFF)
if(USE_EVALUATION_POINT_TYPE)
    add_compile_definitions(USE_EVALUATION_POINT_TYPE)
endif()

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  rosbag 
)

find_package(TBB REQUIRED COMPONENTS tbb)
message(STATUS "The value of TBB_FOUND is: ${TBB_FOUND}")
message(STATUS "The value of TBB_INCLUDE_DIR is: ${TBB_INCLUDE_DIR}")
message(STATUS "The value of TBB_LIBRARY_DIRS is: ${TBB_LIBRARY_DIRS}")
message(STATUS "The value of TBB_LIBRARIES is: ${TBB_LIBRARIES}")

find_package(PCL REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${TBB_LIBRARY_DIRS}
)

# Online mos AWV_MOS alone
add_executable(run_online_mos 
  src/run_online_mos.cpp
  src/AWV_MOS.cpp
  )
target_link_libraries(run_online_mos ${catkin_LIBRARIES} ${PCL_LIBRARIES} TBB::tbb)

# Static map construction
add_executable(run_static_mapping 
  src/run_static_mapping.cpp
  src/AWV_MOS.cpp
  )
target_link_libraries(run_static_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} TBB::tbb)

# MOS evaluation
add_executable(mos_evaluation src/mos_evaluation.cpp)
target_link_libraries(mos_evaluation ${catkin_LIBRARIES} ${PCL_LIBRARIES})