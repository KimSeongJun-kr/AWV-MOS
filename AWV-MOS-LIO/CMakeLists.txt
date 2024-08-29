cmake_minimum_required(VERSION 2.8.3)
project(awv_mos_lio)

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
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  rosbag 
  image_transport
  awv_mos
)

find_package(TBB REQUIRED COMPONENTS tbb)
message(STATUS "The value of TBB_FOUND is: ${TBB_FOUND}")
message(STATUS "The value of TBB_INCLUDE_DIR is: ${TBB_INCLUDE_DIR}")
message(STATUS "The value of TBB_LIBRARY_DIRS is: ${TBB_LIBRARY_DIRS}")
message(STATUS "The value of TBB_LIBRARIES is: ${TBB_LIBRARIES}")

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  cv_bridge 
  image_transport 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  awv_mos
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${TBB_LIBRARY_DIRS}
)

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam TBB::tbb awv_mos)

# Feature Association
add_executable(${PROJECT_NAME}_FeatureExtractionModule src/FeatureExtractionModule.cpp)
add_dependencies(${PROJECT_NAME}_FeatureExtractionModule ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_FeatureExtractionModule ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} TBB::tbb)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam TBB::tbb)