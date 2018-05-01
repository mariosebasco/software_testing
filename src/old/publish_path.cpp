/*
 *
 *
 *
 *
 */

#include <iostream>
#include <fstream>
#include <string>

#include <math.h>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

std::ofstream outFile;
bool not_finished;


/***********************************************************************
 *                                                                     *
 *                      ROS CALLBACK FUNCTION                          *
 *                                                                     *
 *********************************************************************/
void callback(nav_msgs::Odometry msg) {
  static double prev_x = 0.0;
  static double prev_y = 0.0;
  double curr_x = msg.pose.pose.position.x;
  double curr_y = msg.pose.pose.position.y;
  float increment;
  int num_incs;

  // std::cout << "received message" << std::endl;
  // outFile << std::fixed << curr_x << std::endl;
  // outFile << std::fixed << curr_y << std::endl;

  // printf("%f\n", curr_x);
  // printf("%f\n", curr_y);
  
  // return;
  
  if(!prev_x) {//if first point add to path file
    std::cout << "received first message" << std::endl;
    outFile << std::fixed << curr_x << std::endl;
    outFile << std::fixed << curr_y << std::endl;

  }
  else { //interpolate if points are far apart, then add to file
    std::cout << "received another message" << std::endl;
    if(abs(curr_x - prev_x) > 1.0 || abs(curr_y - prev_y) > 1.0) {
      num_incs = (abs(curr_x - prev_x) > abs(curr_y - prev_y)) ? ceil(int(abs(curr_x - prev_x))) : ceil(int(abs(curr_y - prev_y)));
      increment = 1.0/num_incs;
      // std::cout << num_incs << std::endl;
      // std::cout << increment << std::endl;
      for(int i = 1; i <= num_incs; i++) {
	outFile << std::fixed << prev_x + i*increment*(curr_x - prev_x) << std::endl;
	outFile << std::fixed << prev_y + i*increment*(curr_y - prev_y) << std::endl;
      }
    }
    else {
      outFile << std::fixed << curr_x << std::endl;
      outFile << std::fixed << curr_y << std::endl;

    }
    
  }
  prev_x = curr_x;
  prev_y = curr_y;
}

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char *argv[]) {

  //ros variables
  ros::init(argc, argv, "publish_path");
  ros::NodeHandle nh;
  ros::Subscriber utm_sub;
  utm_sub = nh.subscribe("odom", 1, callback);

  outFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");

  not_finished = true;
  //ros::Rate loop_rate(1000);
  while(ros::ok()) {
    ros::spinOnce();
    //loop_rate.sleep();
  }
  
  outFile.close();
  return 0;
}
				 
