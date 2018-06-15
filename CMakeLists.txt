cmake_minimum_required(VERSION 2.8.3)
project(testing)

#add_compile_options(-std=c++11)

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
  EsrTrack.msg
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
  include/testing
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  vncpplib/include
  roboteQ/include
  utm_lib
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

#motor controller
add_library(motor_control_library
  roboteQ/RoboteqDevice.cpp
  )

add_executable(ugv_control
  src/roboteq_driver.cpp
  )

target_link_libraries(ugv_control
  ${catkin_LIBRARIES}
  motor_control_library
  )

#encoder to odom
add_executable(encoder_to_odom
  src/encoder_to_odom.cpp
  )

target_link_libraries(encoder_to_odom
  ${catkin_LIBRARIES}
  )

add_dependencies(encoder_to_odom
  testing_generate_messages_cpp
  )

#publish tf data
add_executable(tf_publisher
  src/tf_pub.cpp
  )

target_link_libraries(tf_publisher
  ${catkin_LIBRARIES}
  )

#testing stuff
add_library(nav_lib
  nav_lib/PeriodicTask.cpp
  nav_lib/ThreadTask.cpp
  nav_lib/AperiodicTask.cpp
  nav_lib/track.cpp
  nav_lib/collision.cpp
  nav_lib/video.cpp
  nav_lib/transform_gps.cpp
  nav_lib/dwa.cpp
  )

add_executable(state_controller
  src/state_controller.cpp
  )

target_link_libraries(state_controller
  ${catkin_LIBRARIES}
  nav_lib
  )

#publish gps long and lat from received path
add_executable(convert_gps
  src/convert_gps.cpp
  )

target_link_libraries(convert_gps
  ${catkin_LIBRARIES}
  )

#calibrate imu offset
add_executable(calibration_test
  src/calibration_test.cpp
  )

target_link_libraries(calibration_test
  ${catkin_LIBRARIES}
  nav_lib
  )


#add exec for radar
# add_executable(radar_exec
#   src/radar.cpp
#   )

# target_link_libraries(radar_exec
#   ${catkin_LIBRARIES}
#   )

# add_dependencies(encoder_to_odom
#   testing_generate_messages_cpp
#   )


