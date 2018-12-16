cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES
  CATKIN_DEPENDS roscpp rospy
# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(image_viewer src/image_viewer.cpp)
target_link_libraries(image_viewer ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(image_viewer_node src/image_viewer_node.cpp)
target_link_libraries(image_viewer_node ${catkin_LIBRARIES} image_viewer)