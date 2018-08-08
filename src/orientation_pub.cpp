/*
 *
 *
 *
 *
 *
 */

#include <stdio>
#include <math.h>
#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

sensor_msgs::Imu imu_data;
nav_msgs::Odometry gps_data;

void ImuCB(sensor_msgs::Imu msg);
void GpsCB(nav_msgs::Odometry msg);

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "orientation_node");
  ros::NodeHandle nh;
  ros::Subscriber imu_sub = nh.subscribe("imu_data", 1, ImuCB);
  ros::Subsriber gps_sub = nh.subscribe("odom/gps_raw", 1, GpsCB);
  ros::Publisher orientation_pub = nh.advertise<sensor_msgs::Imu>("imu_data_filtered", 1);

  ros::Rate loop_rate(100);
  while(ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}


void GpsCB(nav_msgs::Odometry msg) {
  

}
