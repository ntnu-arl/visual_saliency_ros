cmake_minimum_required(VERSION 2.8.3)
project(visual_saliency)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
	cv_bridge
	image_transport
	image_geometry
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

message(${OpenCV_DIR})

include_directories(
  include
  include/saliency_detection
  VOCUS2/src
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  $(Boost_INCLUDE_DIRS)
  )

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS system_lib
)

# Saliency detection
add_executable(saliencyDetectionVOCUS2 src/saliency_detection/saliencyDetectionVOCUS2.cpp VOCUS2/src/VOCUS2.cpp)
target_link_libraries(saliencyDetectionVOCUS2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
