/*
 *
 * given state of the vehicle:
 * - propagate the dynamics forward and check for collision
 * - if collision is going to occur return TRUE, else FALSE
 *
 *
 */


#ifndef COLLISION_H
#define COLLISION_H

#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <vector>

class Collision {
 public:
    
  Collision();
  bool Task(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact);
  bool Task(float move_time, float resolution);
  bool CostmapCheckPoint(float _x_pos, float _y_pos);
  void UpdateCallbacks();

 private:
  float VEHICLE_WIDTH, VEHICLE_LENGTH;
  bool received_map;
  bool received_odom;

  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;
  tf::Quaternion odom_quat;
  nav_msgs::Odometry odom_msg;
  nav_msgs::OccupancyGrid costmap;
  nav_msgs::OccupancyGrid myCostmap;

  ros::Publisher costmap_pub;
  
  bool PropagateState(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact);
  bool CostmapCheck(float _x_pos, float _y_pos, float _theta_pos);
  void OdomCallback(const nav_msgs::Odometry &msg);
  void CostmapCallback(const nav_msgs::OccupancyGrid &msg);
  
  inline double WrapAngle(double angle);
};

#endif
