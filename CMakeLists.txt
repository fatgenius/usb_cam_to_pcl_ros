cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  cv_bridge
  sensor_msgs
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(tf REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${tf_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(image_to_pointcloud_converter src/image_to_pointcloud_converter.cpp)

target_link_libraries(image_to_pointcloud_converter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf_LIBRARIES}
)

install(TARGETS image_to_pointcloud_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
