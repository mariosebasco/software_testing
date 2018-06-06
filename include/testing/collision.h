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
  bool received_map;
  bool received_odom;
    
  Collision();
  void Init(double _move_time_input, double _resolution_input);
  bool Task(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact);
  bool Task(float move_time, float resolution);

 private:
  double move_time, resolution;
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber costmap_sub;
  nav_msgs::Odometry odom_msg;
  tf::Quaternion odom_quat;
  nav_msgs::OccupancyGrid costmap;
  nav_msgs::OccupancyGrid myCostmap;

  ros::Publisher costmap_pub;
  
  bool propagateState(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact);
  bool costmapCheck(float _x_pos, float _y_pos, float _theta_pos);
  void odomCallback(nav_msgs::Odometry msg);
  void costmapCallback(nav_msgs::OccupancyGrid msg);

  inline double wrapAngle(double angle);
};

#endif
