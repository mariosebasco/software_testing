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
Collision::Collision() : PeriodicTask() {
  received_map = false;
  received_odom = false;
  odom_sub = nh.subscribe("local/odom", 1, &Collision::odomCallback, this);
  costmap_sub = nh.subscribe("costmap_node/costmap/costmap", 1, &Collision::costmapCallback, this);
}

/***********************************************************************
 *                                                                     *
 *                      INIT                                           *
 *                                                                     *
 *********************************************************************/
int Collision::Init(char *name, double rate, int priority, double move_time_input, double resolution_input) {
  double actualRate;
  move_time = move_time_input;
  resolution = resolution_input;
  
  return PeriodicTask::Init(name, rate, &actualRate, priority);
}


/***********************************************************************
 *                                                                     *
 *                           MAIN FUNCTION                             *
 *                                                                     *
 *********************************************************************/
void Collision::Task() {
  //wait for local costmap to be received to start
  ros::spinOnce();
  bool collision;

  if(received_map && received_odom) {
    //propagate the state forward for del_t to obtain new state
    collision = propagateState(move_time, resolution);

    if(collision) {StateController::collision_state = CRASH;}
  }
}


/***********************************************************************
 *                                                                     *
 *                      PROPAGATE THE STATE                            *
 *                                                                     *
 *********************************************************************/
bool Collision::propagateState(double move_time, double resolution) {
  double curr_x, curr_y, curr_theta;
  double forward_vel, vel_x, vel_y, vel_theta;
  double next_x, next_y, next_theta;

  double del_t = 0.1;
  bool collision = false;
  int check_collision = int(resolution / del_t);
  
  //get data
  curr_x = odom_msg.pose.pose.position.x;
  curr_y = odom_msg.pose.pose.position.y;
  curr_theta = getYaw(odom_quat);
  curr_theta = curr_theta < 0 ? (2*M_PI + curr_theta) : curr_theta;

  vel_x = odom_msg.twist.twist.linear.x;
  vel_y = odom_msg.twist.twist.linear.y;
  vel_theta = odom_msg.twist.twist.angular.z;

  forward_vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2));

  //find the state
  next_x = curr_x + vel_x*del_t;
  next_y = curr_y + vel_y*del_t;
  next_theta = curr_theta + vel_theta*del_t;
  next_theta = wrapAngle(next_theta);
  next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
  vel_x = forward_vel*cos(next_theta);
  vel_y = forward_vel*sin(next_theta);

  int iterations = int(move_time / del_t); 
  int count = 1;
  
  for(int i = 1; i < iterations; i++) {
    next_x = next_x + vel_x*del_t;
    next_y = next_y + vel_y*del_t;
    next_theta = next_theta + vel_theta*del_t;
    next_theta = wrapAngle(next_theta);
    next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
    vel_x = forward_vel*cos(next_theta);
    vel_y = forward_vel*sin(next_theta);

    //store the propagated state
    positionStruct.x_pos = next_x;
    positionStruct.y_pos = next_y;
    positionStruct.theta = next_theta;

    count += 1;
    
    if(count == check_collision) {  
      collision = costmapCheck();
      if(collision) {
	ROS_INFO("collision detected in %f seconds", (i*0.1 + 0.1));
	return true;
      }
      count = 0;
    }
  }

  return false;
}


/***********************************************************************
 *                                                                     *
 *                      CHECK COSTMAP                                  *
 *                                                                     *
 *********************************************************************/
bool Collision::costmapCheck() {
  //First we need a list of all the x, y points we are concerned with in checking for collision
  //we are going to take points every 5cm along the body in the local frame

  int grid_cell, grid_cell_y, grid_cell_x,  cell_value;
  double origin_x = costmap.info.origin.position.x;
  double origin_y = costmap.info.origin.position.y;
  
  double vehicle_width = 0.4;
  double vehicle_length = 0.85;
  double x_pos = positionStruct.x_pos;
  double y_pos = positionStruct.y_pos;
  double theta_pos = positionStruct.theta;

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
    x_points_glob[i] = x_pos + x_points[i]*cos(theta_pos) - y_points[i]*sin(theta_pos);
    y_points_glob[i] = y_pos + x_points[i]*sin(theta_pos) + y_points[i]*cos(theta_pos);
  }

  //and now transform to cells on the costmap and check if filled
  for(int i = 0; i < vector_length; i++) {
    grid_cell_y = int(fabs(origin_y - y_points_glob[i])/0.05)*200;
    grid_cell_x = int(fabs(origin_x - x_points_glob[i])/0.05); 
    grid_cell= grid_cell_y + grid_cell_x;

    cell_value = costmap.data[grid_cell];

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





