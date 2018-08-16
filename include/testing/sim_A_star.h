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
#include <atomic>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "AperiodicTask.h"
#include "testing/Path_msg.h"

class A_star: public AperiodicTask {
public:
  A_star();
  int Init();

private:
  bool received_odom,  received_costmap, received_path;
  int GOAL_X, GOAL_Y;
  float VEHICLE_WIDTH;
  
  ros::NodeHandle nh;
  ros::Subscriber costmap_sub, path_sub, odom_sub;
  ros::Publisher path_pub, rviz_path_pub;

  nav_msgs::Odometry odom_msg;
  nav_msgs::OccupancyGrid costmap;
  testing::Path_msg path_msg;

  void OdomCB(const nav_msgs::Odometry &msg);
  void CostmapCB(const nav_msgs::OccupancyGrid &msg);
  void PathCB(const testing::Path_msg &msg);
  bool CheckCostmap(int _x, int _y);
  void Task();
};


#endif
