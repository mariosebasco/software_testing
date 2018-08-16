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
  odom_sub = nh.subscribe("local/odom", 1, &Collision::OdomCallback, this);
  costmap_sub = nh.subscribe("costmap_node/costmap/costmap", 1, &Collision::CostmapCallback, this);

  costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("my_costmap", 1);

  //nh.getParam("/vehicle_width", vehicle_width);
  //nh.getParam("/vehicle_length", vehicle_length);
  VEHICLE_WIDTH = 0.4;
  VEHICLE_LENGTH = 0.85;
}


/***********************************************************************
 *                                                                     *
 *                           MAIN FUNCTION                             *
 *                                                                     *
 *********************************************************************/
bool Collision::Task(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact) {
  float move_time, resolution, trans_vel, rot_vel;
  bool collision;

  while(!received_map || !received_odom) {
    UpdateCallbacks();
  }

  move_time = _move_time;
  resolution = _resolution;
  trans_vel = _trans_vel;
  rot_vel = _rot_vel;
    
  //propagate the state forward for del_t to obtain new state
  collision = PropagateState(move_time, resolution, trans_vel, rot_vel, time_to_impact);
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

  while(!received_map || !received_odom) {
    UpdateCallbacks();
  }
  
  move_time = _move_time;
  resolution = _resolution;
  trans_vel = odom_msg.twist.twist.linear.x;
  rot_vel = odom_msg.twist.twist.angular.z;
  float time_to_impact;
  
  //propagate the state forward for del_t to obtain new state
  collision = PropagateState(move_time, resolution, trans_vel, rot_vel, time_to_impact);
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
bool Collision::PropagateState(float _move_time, float _resolution, float _trans_vel, float _rot_vel, float &time_to_impact) {

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

  int iterations = int(move_time / resolution); 
  
  //propagate the state
  if(rot_vel == 0.0) {
    next_theta = curr_theta;
    for(int i = 1; i <= iterations; i++) {
      next_x = curr_x + trans_vel*cos(curr_theta)*i*resolution;
      next_y = curr_y + trans_vel*sin(curr_theta)*i*resolution;

      collision = CostmapCheck(next_x, next_y, next_theta);
      if(collision) {
  	time_to_impact = i*resolution;
  	//ROS_INFO("collision detected in %f seconds", time_to_impact);
  	return true;
      }
    }
  }
  else {
    float radius = trans_vel / rot_vel;
    float circle_x = curr_x - radius*sin(curr_theta);
    float circle_y = curr_y + radius*cos(curr_theta);
  
    for(int i = 1; i <= iterations; i++) {
      next_theta = curr_theta + rot_vel*i*resolution;
      next_x = circle_x + radius*sin(next_theta);
      next_y = circle_y - radius*cos(next_theta);

      // printf("radius: %f\n", radius);
      // printf("circle x: %f\n", circle_x);
      // printf("circle y: %f\n\n", circle_y);

      // printf("next theta: %f\n", next_theta);
      // printf("next x: %f\n", next_x);
      // printf("next y: %f\n", next_y);
      // std::cout << "********************" << std::endl << std::endl;
      
      collision = CostmapCheck(next_x, next_y, next_theta);
      if(collision) {
  	time_to_impact = i*resolution;
  	//ROS_INFO("collision detected in %f seconds", time_to_impact);
  	return true;
      }
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
bool Collision::CostmapCheck(float _x_pos, float _y_pos, float _theta_pos) {
  //First we need a list of all the x, y points we are concerned with in checking for collision
  //we are going to take points every 5cm along the body in the local frame and convert it to the global frame
  //then we'll see if those points are occupied in the grid

  int grid_cell, grid_cell_y, grid_cell_x,  cell_value, map_height, map_width;
  double origin_x, origin_y;
  float map_resolution, x_pos, y_pos, theta_pos;
  origin_x = costmap.info.origin.position.x;
  origin_y = costmap.info.origin.position.y;
  map_resolution = costmap.info.resolution;
  map_height = costmap.info.height;
  map_width = costmap.info.width;
  
  x_pos = _x_pos;
  y_pos = _y_pos;
  theta_pos = _theta_pos;

  if(int(fabs((origin_x - x_pos)/map_resolution)) > map_width) return false;
  if(int(fabs((origin_y + y_pos)/map_resolution)) > map_height) return false;

  std::vector<double> x_points(int(ceil(VEHICLE_WIDTH/0.05)) + 1, VEHICLE_LENGTH/2.0);
  std::vector<double> y_points(int(ceil(VEHICLE_WIDTH/0.05)) + 1, -VEHICLE_WIDTH/2.0);
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
    //std::cout << "grid cell " << grid_cell << std::endl;

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
 *                      CHECK COSTMAP POINT                            *
 *           Check a single x, y point on the costmap                  *
 *                                                                     *
 *                                                                     *
 *********************************************************************/
bool Collision::CostmapCheckPoint(float _x_pos, float _y_pos) {
  int grid_cell, grid_cell_y, grid_cell_x,  cell_value, map_height, map_width;
  double origin_x, origin_y;
  float map_resolution, x_pos, y_pos;

  x_pos = _x_pos;
  y_pos = _y_pos;
  
  origin_x = costmap.info.origin.position.x;
  origin_y = costmap.info.origin.position.y;
  map_resolution = costmap.info.resolution;
  map_height = costmap.info.height;
  map_width = costmap.info.width;
  
  if(int(fabs((origin_x - x_pos)/map_resolution)) > map_width) return false;
  if(int(fabs((origin_y + y_pos)/map_resolution)) > map_height) return false;
  
  grid_cell_y = int(fabs(origin_y + y_pos)/map_resolution)*map_height;
  grid_cell_x = int(fabs(origin_x - x_pos)/map_resolution); 
  grid_cell= grid_cell_y + grid_cell_x;
  
  cell_value = costmap.data[grid_cell];
  
  if(cell_value > 0) {
    return true;
  }

  return false;
}

/***********************************************************************
 *                                                                     *
 *                      SUBSCRIBER CALLBACK ODOM                       *
 *                                                                     *
 *********************************************************************/
void Collision::OdomCallback(const nav_msgs::Odometry &msg) {
  received_odom = true;
  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}


/***********************************************************************
 *                                                                     *
 *                  SUBSCRIBER CALLBACK COSTMAP                        *
 *                                                                     *
 *********************************************************************/
void Collision::CostmapCallback(const nav_msgs::OccupancyGrid &msg) {
  received_map = true;
  costmap = msg;
  myCostmap = msg;
}

/***********************************************************************
 *                                                                     *
 *                  UPDATE CALLBACK FUNCTIONS                          *
 *                                                                     *
 *********************************************************************/
void Collision::UpdateCallbacks() {
  ros::spinOnce();
}


/***********************************************************************
 *                                                                     *
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double Collision::WrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}





