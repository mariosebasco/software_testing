cmake_minimum_required(VERSION 2.8.3)
project(testing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  joint_state_publisher
  robot_state_publisher
  rviz
  xacro
  gazebo_ros
  message_generation
)

add_message_files(
  FILES
  encoder_msg.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS message_runtime
 #INCLUDE_DIRS include
 #LIBRARIES testing
 #CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
 #DEPENDS Boost
 )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  vncpplib/include
  roboteQ/include
)

#IMU
add_executable(imu
  src/imu.cpp)

add_library(imu_lib
  vncpplib/src/arch/linux/vncp_services.c
  vncpplib/src/vndevice.c
  vncpplib/src/vn100.c
  )

target_link_libraries(imu
  ${catkin_LIBRARIES}
  imu_lib
  )

#
add_executable(odometry_publisher
  src/move_urdf.cpp
  )

target_link_libraries(odometry_publisher
  ${catkin_LIBRARIES})

#
add_executable(cmd_vel_publisher
  src/ugv_control.cpp
  )

target_link_libraries(cmd_vel_publisher
  ${catkin_LIBRARIES})


#motor controller
add_library(motor_control_library
  roboteQ/RoboteqDevice.cpp
  )

add_executable(motor_control
  src/roboteq_driver.cpp
  )

target_link_libraries(motor_control
  ${catkin_LIBRARIES}
  motor_control_library
  )


