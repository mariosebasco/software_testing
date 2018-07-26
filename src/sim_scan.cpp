/*
 *
 * CONVERT WIDE 270 LMS111 SCAN TO SMALLER ANGLE SCAN 
 *
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <math.h>

sensor_msgs::LaserScan my_scan;
float ANGLE_MIN = -M_PI/2.0;
float ANGLE_MAX = M_PI/2.0;
ros::Publisher laser_pub;

void LaserCB(sensor_msgs::LaserScan msg) {
  my_scan = msg;
  for (int i = 0; i < msg.ranges.size(); i++) {
    if(msg.ranges[i] < 50.0 && msg.ranges[i] > 0.0) {
      my_scan.intensities[i] = 1000.0;
    }
  }

  laser_pub.publish(my_scan);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_scan");
  ros::NodeHandle nh;
  ros::Subscriber laser_sub = nh.subscribe("ugv/laser/scan", 1, LaserCB);
  laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);

  
  ros::spin();

  return 0;
}
