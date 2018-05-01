/*
 *
 *
 *
 *
 *
 */

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

bool should_start = false;
nav_msgs::Odometry odom_msg;
tf::Quaternion odom_quat;

void odomCallback(nav_msgs::Odometry msg) {
  should_start = true;

  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}


int main(int argc, char *argv[]) {

  //initialize ROS variables
  ros::init(argc, argv, "calibration_node");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub= nh.subscribe("odom", 1, odomCallback);
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  geometry_msgs::Twist cmd_vel;
  
  std::cout << "Starting calibration..." << std::endl;
    
  //wait for odom data
  while(!should_start) {
    ros::spinOnce();
  }
  double init_northing = odom_msg.pose.pose.position.x;
  double init_easting = odom_msg.pose.pose.position.y;
    
  cmd_vel.linear.x = 0.5;
  cmd_vel.angular.z = 0.0;
  vel_pub.publish(cmd_vel);
  
  ros::Duration(10).sleep();

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub.publish(cmd_vel);

  ros::spinOnce();
    
  double final_northing = odom_msg.pose.pose.position.x;
  double final_easting = odom_msg.pose.pose.position.y;

  double del_northing = final_northing - init_northing;
  double del_easting = final_easting - init_easting;

  if(del_northing == 0 && del_easting == 0) {
    std::cout << "Calibration failed" << std::endl;
    return 0;
  }

  double theta_gps = atan2(del_easting, del_northing);
  double theta_imu = getYaw(odom_quat);

  double imu_drift = - theta_imu + theta_gps;
  std::cout << "imu drift: " << imu_drift << std::endl;
  std::cout << "Calibration finished" << std::endl;

  nh.setParam("/calibration_value", imu_drift);
  
  return 0;
}
