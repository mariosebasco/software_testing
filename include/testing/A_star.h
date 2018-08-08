/*
 *
 *
 *
 *
 *
 */

#ifndef A_STAR
#define A_STAR

#include <iostream>
#include <math.h>
#include <set>
#include <stdio.h>
#include <vector>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "AperiodicTask.h"

class A_star: public AperiodicTask {
public:
  float GOAL_X_LOCAL, GOAL_Y_LOCAL;
  bool INTERRUPT;
  
  A_star();
  int Init();

private:
  bool received_odom, received_costmap;
  int GOAL_X, GOAL_Y;
  float VEHICLE_WIDTH;
  
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;
  ros::Publisher path_pub;

  nav_msgs::Odometry odom_msg;
  nav_msgs::OccupancyGrid costmap;

  void OdomCB(nav_msgs::Odometry msg);
  void CostmapCB(nav_msgs::OccupancyGrid msg);
  bool CheckCostmap(int _x, int _y);
  void Task();
};


#endif
