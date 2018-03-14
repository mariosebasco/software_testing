/*
 *
 * given state of the vehicle:
 * - propagate the dynamics forward and check for collision
 * - if collision is going to occur return TRUE, else FALSE
 *
 *
 */

#include "collision.h"

/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
Collision::Collision() {
  should_start = false;
  
  odom_sub = nh.subscribe("odom", 1, &Collision::odomCallback, this);
  costmap_sub = nh.subscribe("move_base/local_costmap/costmap", 1, &Collision::costmapCallback, this);
}


/***********************************************************************
 *                                                                     *
 *                           MAIN FUNCTION                             *
 *                                                                     *
 *********************************************************************/
bool Collision::willCollide() {
  //wait for local costmap to be received to start
  ros::spinOnce();

  if(should_start) {
    bool collision;
    float move_time = 2.0;
    
    //propagate the state forward for del_t to obtain new state
    propagateState(move_time);

    //Does the front bumper of the vehicle doincide with high costmap cells
    collision = costmapCheck();
    
    return collision;
  }
}


/***********************************************************************
 *                                                                     *
 *                      PROPAGATE THE STATE                            *
 *                                                                     *
 *********************************************************************/
void Collision::propagateState(float move_time) {
  float curr_x, curr_y, curr_theta;
  float vel_x, vel_y, vel_theta;
  float next_x, next_y, next_theta;
  float del_t = 0.1;
  
  //get data
  curr_x = odom_msg.pose.pose.position.x;
  curr_y = odom_msg.pose.pose.position.y;
  curr_theta = getYaw(odom_quat);
  curr_theta = curr_theta < 0 ? (2*M_PI + curr_theta) : curr_theta;
  
  vel_x = odom_msg.twist.twist.linear.x;
  vel_y = odom_msg.twist.twist.linear.y;
  vel_theta = odom_msg.twist.twist.angular.z;

  //find the next state
  next_x = curr_x + vel_x*cos(curr_theta)*del_t;
  next_y = curr_y + vel_y*sin(curr_theta)*del_t;
  next_theta = curr_theta + vel_theta*del_t;
  next_theta = wrapAngle(next_theta);
  next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;

  int iterations = int(move_time / del_t); 

  for(int i = 1; i < iterations; i++) {
    next_x = next_x + vel_x*cos(next_theta)*del_t;
    next_y = next_y + vel_y*sin(next_theta)*del_t;
    next_theta = next_theta + vel_theta*del_t;
    next_theta = wrapAngle(next_theta);
    next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
  }  
  
  //store the propagated state
  positionStruct.x_pos = next_x;
  positionStruct.y_pos = next_x;
  positionStruct.theta = next_theta;
  
}


/***********************************************************************
 *                                                                     *
 *                      CHECK COSTMAP                                  *
 *                                                                     *
 *********************************************************************/
bool Collision::costmapCheck() {
  //First we need a list of all the x, y points we are concerned with in checking for collision
  //we are going to take points every 5cm along the body in the local frame
  int grid_cell, cell_value;
  float origin_x = costmap.info.origin.position.x;
  float origin_y = costmap.info.origin.position.y;
  
  float vehicle_width = 0.3;
  float vehicle_length = 0.5;
  float x_var = positionStruct.x_pos;
  float y_var = positionStruct.y_pos;
  float theta_var = positionStruct.theta;

  std::vector<float> x_points(int(vehicle_width/0.05), -vehicle_width/2.0);
  std::vector<float> y_points(int(vehicle_width/0.05), vehicle_length/2.0);
  int vector_length = x_points.size();

  std::vector<float> x_points_glob(vector_length, 0.0);
  std::vector<float> y_points_glob(vector_length, 0.0);
  
  for(int i = 0; i < vector_length; i++) {
    x_points[i] = x_points[i] + 0.05*i;
  }

  //now that we have the x and y points in the frame of the vehicle
  //We transform to the odom frame
  for(int i = 0; i < vector_length; i++) {
    x_points_glob[i] = x_var + x_points[i]*cos(theta_var) - y_points[i]*sin(theta_var);
    y_points_glob[i] = y_var + x_points[i]*sin(theta_var) + y_points[i]*cos(theta_var);
  }

  //and now transform to cells on the costmap and check if filled
  for(int i = 0; i < vector_length; i++) {
    grid_cell= int(abs(origin_y - y_points_glob[i])*200/0.05 + abs(origin_x - x_points_glob[i])/0.05);
    cell_value = costmap.data[grid_cell];
    if(cell_value >= 50) {return true;}
  }

  return false;
}


/***********************************************************************
 *                                                                     *
 *                      SUBSCRIBER CALLBACK ODOM                       *
 *                                                                     *
 *********************************************************************/
void Collision::odomCallback(nav_msgs::Odometry msg) {
  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  
}


/***********************************************************************
 *                                                                     *
 *                  SUBSCRIBER CALLBACK COSTMAP                        *
 *                                                                     *
 *********************************************************************/
void Collision::costmapCallback(nav_msgs::OccupancyGrid msg) {
  should_start = true;
  costmap = msg;
}

/***********************************************************************
 *                                                                     *
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double Collision::wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}





