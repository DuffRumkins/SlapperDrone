cmake_minimum_required(VERSION 3.0.2)
project(slapper_vision)


find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  nodelet
  tf2_ros
  geometry_msgs
  message_generation
  genmsg
)

find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)

#Set flag for using threads
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Perch.msg
#   Message1.msg
#   Message2.msg
)



# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/cylinder_detector.cfg
  cfg/coefficient_cylinder_detector.cfg
  cfg/cluster_cylinder_detector.cfg
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES line_segment_detector
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs nodelet
#  DEPENDS system_lib
)


include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cylinder_detector src/cylinder_detector.cpp)
target_link_libraries(cylinder_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(cylinder_detector ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

