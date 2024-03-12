cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_process)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system -DGTSAM_USE_SYSTEM_EIGEN=ON")

find_package(catkin REQUIRED COMPONENTS
   tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}

)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}

)

add_executable(cloud_processing src/point_cloud_process.cpp)
target_link_libraries(cloud_processing ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

add_executable(cloud_translator src/pcd_cloud_translator.cpp)
target_link_libraries(cloud_translator ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
