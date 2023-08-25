cmake_minimum_required(VERSION 3.0.2)
project(slapper_control)

find_package(catkin REQUIRED dynamic_reconfigure COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf2_ros
  message_generation
  slapper_vision
)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/pid_controller.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES slapper_control
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy std_msgs slapper_vision
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




add_executable(offboard_takeoff src/offboard_takeoff.cpp)
target_link_libraries(offboard_takeoff ${catkin_LIBRARIES})

add_executable(body_frame_transform src/body_frame_transform.cpp)
target_link_libraries(body_frame_transform ${catkin_LIBRARIES})

add_executable(track_cylinder src/track_cylinder.cpp)
target_link_libraries(track_cylinder ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(approach_perch src/approach_perch.cpp)
target_link_libraries(approach_perch ${catkin_LIBRARIES})

add_executable(local_position_controller src/local_position_controller.cpp)
target_link_libraries(local_position_controller ${catkin_LIBRARIES})

add_executable(random_setpoint src/random_setpoint.cpp)
target_link_libraries(random_setpoint ${catkin_LIBRARIES})

add_executable(planning_box src/planning_box.cpp)
target_link_libraries(planning_box ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller ${PROJECT_NAME}_gencfg)

add_executable(gazebo_mocap_odom src/gazebo_mocap_odom.cpp)
target_link_libraries(gazebo_mocap_odom ${catkin_LIBRARIES})

add_executable(gazebo_odom_remap src/gazebo_odom_remap.cpp)
target_link_libraries(gazebo_odom_remap ${catkin_LIBRARIES})