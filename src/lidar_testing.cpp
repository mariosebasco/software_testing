#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


sensor_msgs::LaserScan my_scan;


void LaserCB(sensor_msgs::LaserScan msg) {
  //std::cout << "range size" << msg.ranges.size() << std::endl;
  my_scan = msg;
  float angle;
  
  for (int i = 0; i < (msg.ranges.size()); i++) {
    angle  = msg.angle_min + msg.angle_increment*i;
    if (angle > -M_PI/2.0 && angle < 0.0) {
      // std::cout << "range " << msg.ranges[i] << std::endl;
      // std::cout << "angle " << (msg.angle_min + msg.angle_increment*i)*180.0/M_PI << std::endl;
    }
    else {
      my_scan.ranges[i] = 0.0;
      my_scan.intensities[i] = 0.0;
    }
  }
  std::cout << "***************************" << std::endl;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_test_node");
  ros::NodeHandle nh;
  ros::Subscriber lidar_sub = nh.subscribe("scan", 1, LaserCB);
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan_new", 1000);

  ros::Rate loop_rate(100);
  
  while(ros::ok()) {
    ros::spinOnce();
    laser_pub.publish(my_scan);
  }

  return 0;
}
