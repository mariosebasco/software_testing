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
Collision::Collision() {//: PeriodicTask() {
  received_map = false;
  received_odom = false;
  odom_sub = nh.subscribe("local/odom", 1, &Collision::odomCallback, this);
  costmap_sub = nh.subscribe("costmap_node/costmap/costmap", 1, &Collision::costmapCallback, this);

  costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("my_costmap", 1);
}


/***********************************************************************
 *                                                                     *
 *                           MAIN FUNCTION                             *
 *                                                                     *
 *********************************************************************/
bool Collision::Task(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact) {
  float move_time, resolution, trans_vel, rot_vel;
  bool collision;

  while(!received_map && received_odom) {
    ros::spinOnce();
  }

  move_time = _move_time;
  resolution = _resolution;
  trans_vel = _trans_vel;
  rot_vel = _rot_vel;
    
  //propagate the state forward for del_t to obtain new state
  collision = propagateState(move_time, resolution, trans_vel, rot_vel, time_to_impact);
  costmap_pub.publish(myCostmap);
  if(collision) return true;
  return false;

}


/***********************************************************************
 *                                                                     *
 *                           MAIN FUNCTION                             *
 *                                                                     *
 *********************************************************************/
bool Collision::Task(float _move_time, float _resolution) {
  float move_time, resolution, trans_vel, rot_vel;
  bool collision;
  
  while(!received_map && !received_odom) {
    ros::spinOnce();
  }

  move_time = _move_time;
  resolution = _resolution;
  trans_vel = odom_msg.twist.twist.linear.x;
  rot_vel = odom_msg.twist.twist.angular.z;
  float time_to_impact;
  
  //propagate the state forward for del_t to obtain new state
  collision = propagateState(move_time, resolution, trans_vel, rot_vel, time_to_impact);
  costmap_pub.publish(myCostmap);
  if(collision) return true;
  return false;
}


/***********************************************************************
 *                                                                     *
 *                      PROPAGATE THE STATE                            *
 *    We assume constant velocity so that we move along a circle       *
 *       of radius defined by the rot and trans velocities             *
 *                                                                     *
 * resolution: How often along move time we want to check for a        *
 * collsion                                                            *
 * move_time: how far forward to propagate the state                   *
 *********************************************************************/
bool Collision::propagateState(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact) {

  double curr_x, curr_y, curr_theta, next_x, next_y, next_theta;
  float move_time, resolution, trans_vel, rot_vel;
  bool collision = false;

  //get data
  move_time = _move_time;
  resolution = _resolution;
  trans_vel = _trans_vel;
  rot_vel = _rot_vel;

  curr_x = odom_msg.pose.pose.position.x;
  curr_y = odom_msg.pose.pose.position.y;
  curr_theta = getYaw(odom_quat);
  curr_theta = curr_theta < 0 ? (2*M_PI + curr_theta) : curr_theta;
  
  //the radius of the circle we are moving along is
  float radius = trans_vel / rot_vel;
  float circle_x = curr_x - radius*sin(-curr_theta);
  float circle_y = curr_y + radius*cos(-curr_theta);

  int iterations = int(move_time / resolution); 
  
  for(int i = 1; i <= iterations; i++) {
    next_theta = curr_theta + rot_vel*i*resolution;
    next_x = circle_x + radius*sin(-next_theta);
    next_y = circle_y + radius*cos(-next_theta);
        
    collision = costmapCheck(next_x, next_y, next_theta);
      if(collision) {
	time_to_impact = i*resolution + resolution;
	ROS_INFO("collision detected in %f seconds", time_to_impact);
	return true;
      }
  }
  return false;
}


/***********************************************************************
 *                                                                     *
 *                      CHECK COSTMAP                                  *
 * Given x, y, and theta in local frame find out if there is a         *
 * something in that location                                          *
 *                                                                     *
 *********************************************************************/
bool Collision::costmapCheck(float _x_pos, float _y_pos, float _theta_pos) {
  //First we need a list of all the x, y points we are concerned with in checking for collision
  //we are going to take points every 5cm along the body in the local frame and convert it to the global frame
  //then we'll see if those points are occupied in the grid

  int grid_cell, grid_cell_y, grid_cell_x,  cell_value, map_height;
  double origin_x, origin_y;
  float map_resolution, vehicle_width, vehicle_length, x_pos, y_pos, theta_pos, time_to_impact;
  origin_x = costmap.info.origin.position.x;
  origin_y = costmap.info.origin.position.y;
  map_resolution = costmap.info.resolution;
  map_height = costmap.info.height;
  
  nh.getParam("/vehicle_width", vehicle_width);
  nh.getParam("/vehicle_length", vehicle_length);
  
  x_pos = _x_pos;
  y_pos = _y_pos;
  theta_pos = _theta_pos;
    
  std::vector<double> x_points(int(ceil(vehicle_width/0.05)) + 1, vehicle_length/2.0);
  std::vector<double> y_points(int(ceil(vehicle_width/0.05)) + 1, -vehicle_width/2.0);
  int vector_length = x_points.size();

  std::vector<double> x_points_glob(vector_length, 0.0);
  std::vector<double> y_points_glob(vector_length, 0.0);
  
  for(int i = 0; i < vector_length; i++) {
    y_points[i] = y_points[i] + 0.05*i;
  }

  //now that we have the x and y points in the frame of the vehicle
  //We transform to the odom frame
  for(int i = 0; i < vector_length; i++) {
    x_points_glob[i] = (x_pos + x_points[i]*cos(-theta_pos) - y_points[i]*sin(-theta_pos));
    y_points_glob[i] = (y_pos - x_points[i]*sin(-theta_pos) - y_points[i]*cos(-theta_pos));
  }

  //and now transform to cells on the costmap and check if filled
  for(int i = 0; i < vector_length; i++) {
    grid_cell_y = int(fabs(origin_y + y_points_glob[i])/map_resolution)*map_height;
    grid_cell_x = int(fabs(origin_x - x_points_glob[i])/map_resolution); 
    grid_cell= grid_cell_y + grid_cell_x;

    cell_value = costmap.data[grid_cell];
    myCostmap.data[grid_cell] = 100;

    if(cell_value > 0) {
      return true;
    }
  }
  return false;
}


/***********************************************************************
 *                                                                     *
 *                      SUBSCRIBER CALLBACK ODOM                       *
 *                                                                     *
 *********************************************************************/
void Collision::odomCallback(nav_msgs::Odometry msg) {
  received_odom = true;
  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  
}


/***********************************************************************
 *                                                                     *
 *                  SUBSCRIBER CALLBACK COSTMAP                        *
 *                                                                     *
 *********************************************************************/
void Collision::costmapCallback(nav_msgs::OccupancyGrid msg) {
  received_map = true;
  costmap = msg;
  myCostmap = msg;
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





