/*
 * DWA algoirthm is used for obstacle avoidance
 * Given an obstacle in the path wechoose a linear and rotational velocity
 * which maximizes a cost function, its inputs are:
 *    heading to the goal point: you want to be facing the goal point
 *    dist to nearest obstacle in current path: the closer to an obstacle the more you want to turn
 *    velocity (higher velocity is favorable)
 */

#ifndef DWA_H
#define DWA_H

#include <math.h>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "collision.h"
#include "AperiodicTask.h"

class DWA : public AperiodicTask {
 public:
  bool REACHED_GOAL;
  float GOAL_X, GOAL_Y;
  
  struct VelocityStruct {
    float trans_vel;
    float rot_vel;
  };

  DWA(Collision* _collisionObject);
  int Init();
  void Task();

 private:
  bool received_odom;
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Publisher vel_pub;
  nav_msgs::Odometry odom_msg;
  tf::Quaternion odom_quat;

  float max_trans_vel;
  float max_rot_vel;
  float max_trans_acceleration;
  float max_rot_acceleration;

  Collision *collisionObject;
  
  std::vector<VelocityStruct> FindDynamicWindow(VelocityStruct _velocity_struct, float _sim_time, float _resolution);
  float FindOrientationCost(VelocityStruct velocity_struct, float goal_x, float goal_y, float sim_time);
  float FindObstacleCost(VelocityStruct _velocity_struct, float _resolution);
  float FindVelocityCost(VelocityStruct _velocity_struct);
  bool ReachedGoal(float _goal_x, float _goal_y);
  void OdomCallback(nav_msgs::Odometry msg);
  void PublishVel(float trans_vel, float rot_vel);
};

#endif
