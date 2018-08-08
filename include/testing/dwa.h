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
#include <fstream>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"

#include "collision.h"
#include "AperiodicTask.h"
#include "testing/Path_msg.h"

class DWA : public AperiodicTask {
 public:
  bool REACHED_GOAL, INTERRUPT;
  float MAX_DIST_FROM_PATH;
  
  DWA(Collision* _collisionObject);
  int Init();

 private:
  struct VelocityStruct {
    float trans_vel;
    float rot_vel;
  };
  
  bool received_odom, received_path, received_odom_global, received_a_star;
  ros::NodeHandle nh;
  ros::Subscriber odom_sub, odom_sub_global, path_sub, a_star_sub;
  ros::Publisher vel_pub;
  nav_msgs::Odometry odom_msg, odom_msg_global;
  testing::Path_msg path_msg;
  nav_msgs::Path a_star_path;
  tf::Quaternion odom_quat;

  float MAX_TRANS_VEL;
  float MAX_ROT_VEL;
  float MAX_TRANS_ACCELERATION;
  float MAX_ROT_ACCELERATION;
  float SIM_TIME;
  float RESOLUTION;
  
  Collision *collisionObject;
  
  void Task();
  float FindOrientationCost(VelocityStruct _velocity_struct, float _goal_x, float _goal_y, float &dist_to_goal);
  float FindObstacleCost(VelocityStruct _velocity_struct);
  float FindVelocityCost(VelocityStruct _velocity_struct);
  float FindPathCost(VelocityStruct _velocity_struct);
  void OdomCallback(nav_msgs::Odometry msg);
  void OdomGlobalCB(nav_msgs::Odometry msg);
  void AStarCB(nav_msgs::Path msg);
  void PathCB(testing::Path_msg msg);
  void PublishVel(float trans_vel, float rot_vel);
  float FindDistFromPath(float _x1, float _y1, float _x2, float _y2);
};

#endif
