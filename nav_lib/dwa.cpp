/*
 * DWA algoirthm is used for obstacle avoidance
 * Given an obstacle in the path wechoose a linear and rotational velocity
 * which maximizes a cost function, its inputs are:
 *    heading to the goal point: you want to be facing the goal point
 *    dist to nearest obstacle in current path: the closer to an obstacle the more you want to turn
 *    velocity (higher velocity is favorable)
 */


#include "dwa.h"

DWA::DWA(Collision* _collisionObject) : AperiodicTask() {
  collisionObject = _collisionObject;
  
  odom_sub = nh.subscribe("local/odom", 1, &DWA::OdomCallback, this);    
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
  received_odom = false;
  REACHED_GOAL = false;
  GOAL_X = 0.0;
  GOAL_Y = 0.0;

  max_trans_vel = 0.25; //0.25m/s for testing
  max_rot_vel = M_PI/2;
}

int DWA::Init() {
  return AperiodicTask::Init((char *) "localTrackTask", 50);
}


void DWA::Task() {

  //make sure you've received odometry data
  while(!received_odom) {
    ros::spinOnce();
  }

  float orientation_cost, obstacle_cost, velocity_cost, path_cost, resolution;
  float alpha, beta, gamma;
  float optimal_trans_vel, optimal_rot_vel;
  float sim_time;
  float max_path_cost = 0;

  nh.getParam("/max_trans_acceleration", max_trans_acceleration);
  nh.getParam("/max_rot_acceleration", max_rot_acceleration);
  nh.getParam("/sim_time_dwa", sim_time);
  nh.getParam("/resolution", resolution);

  alpha = 0.7;
  beta = 0.2;
  gamma = 0.2;

  VelocityStruct velocity_struct;
  std::vector<VelocityStruct> velocities;
    
  ros::Rate loop_rate(1/sim_time);
    
  while(ros::ok()) {
    //find the set of possible velocities (Dynamic Window)
    velocity_struct.trans_vel = odom_msg.twist.twist.angular.z;
    velocity_struct.rot_vel = odom_msg.twist.twist.linear.x;
    velocities = FindDynamicWindow(velocity_struct, sim_time, resolution);
    if(velocities.empty()) {
      std::cout << "No feasible velocities found" << std::endl;
      return;
    }

    int vector_size = velocities.size();
    for(int i = 0; i < vector_size; i++) {
      //For all possible velocities find the costs
      orientation_cost = FindOrientationCost(velocities[i], GOAL_X, GOAL_Y, sim_time);
      obstacle_cost = FindObstacleCost(velocities[i], resolution);
      velocity_cost = FindVelocityCost(velocities[i]);

      path_cost = alpha*orientation_cost + beta*obstacle_cost + gamma*velocity_cost;
      //update optimal velocity
      if(max_path_cost < path_cost) {
	max_path_cost = path_cost;
	optimal_trans_vel = velocities[i].trans_vel;
	optimal_rot_vel = velocities[i].rot_vel;
      }
    }

    PublishVel(optimal_trans_vel, optimal_rot_vel);
    
    if(ReachedGoal(GOAL_X, GOAL_Y)) {
      PublishVel(0.0, 0.0);
      REACHED_GOAL = true;
      break;
    }
      
    ros::spinOnce();
    loop_rate.sleep();
  }
}



/************************************************************************
 *                  FIND THE DYNAMIC WINDOW                             *
 *  The dynamic window is the set of velocities allowable given         * 
 *  the state. These velocities are calculated from the maximum         *
 *  acceleration and velocity values of the robot, as well as           *
 *  obstacles in the environment                                        *          
 ***********************************************************************/
std::vector<DWA::VelocityStruct> DWA::FindDynamicWindow(VelocityStruct _velocity_struct, float _sim_time, float _resolution) {

  std::vector<VelocityStruct> velocities;
  float resolution, sim_time, time_to_impact;
  VelocityStruct velocity_struct;

  velocity_struct = _velocity_struct;
  resolution = _resolution;
  sim_time = _sim_time;

  float vel_upper_limit = velocity_struct.trans_vel + max_trans_acceleration*sim_time;
  float vel_lower_limit = velocity_struct.trans_vel - max_trans_acceleration*sim_time;
  float rot_upper_limit = velocity_struct.rot_vel + max_rot_acceleration*sim_time;
  float rot_lower_limit = velocity_struct.rot_vel - max_rot_acceleration*sim_time;

  if(vel_upper_limit > max_trans_vel) vel_upper_limit = max_trans_vel;
  if(vel_lower_limit < 0.0) vel_lower_limit = 0.0;
  if(rot_upper_limit > max_rot_vel) rot_upper_limit = max_rot_vel;
  if(rot_lower_limit < -max_rot_vel) rot_lower_limit = -max_rot_vel;
  
  float num_trans_iterations = int((vel_upper_limit - vel_lower_limit)/resolution);
  float num_rot_iterations = int((rot_upper_limit - rot_lower_limit)/resolution);

  float temp_trans_vel;
  float temp_rot_vel;
  VelocityStruct temp_vel_struct;
  
  for(int i = 0; i < num_trans_iterations; i++) {
    for(int j = 0; j < num_rot_iterations; j++) {
      temp_trans_vel = vel_lower_limit + i*resolution;
      temp_rot_vel = rot_lower_limit + j*resolution;
      if(collisionObject->Task(sim_time, resolution, temp_trans_vel, temp_rot_vel, time_to_impact)) {break;}
      temp_vel_struct.trans_vel = temp_trans_vel;
      temp_vel_struct.rot_vel = temp_rot_vel;
      velocities.push_back(temp_vel_struct);
    }
  }

  return velocities;
}

/************************************************************************
 *                  FIND THE ORIENTATION COST                           *
 *  We want to favor velocities whose final orientation will face       * 
 *  the goal. Cost = 1 is directly facing the goal                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindOrientationCost(VelocityStruct _velocity_struct, float _goal_x, float _goal_y, float _sim_time) {

  VelocityStruct velocity_struct;
  float cost, goal_x, goal_y, sim_time, radius, circle_x, circle_y;
  float curr_x, curr_y, curr_theta, next_x, next_y, next_theta;
  
  velocity_struct = _velocity_struct;
  goal_x = _goal_x;
  goal_y = _goal_y;
  sim_time = _sim_time;
  
  //find new pose
  radius = velocity_struct.trans_vel/velocity_struct.rot_vel;
  circle_x = curr_x - radius*sin(-curr_theta);
  circle_y = curr_y + radius*cos(-curr_theta);

  next_theta = curr_theta + velocity_struct.rot_vel*sim_time;
  next_x = circle_x + radius*sin(-next_theta);
  next_y = circle_y + radius*cos(-next_theta);

  float theta_des, theta_curr, angle_error;
  theta_des = atan2((goal_y - next_y), (goal_x - next_x));
  theta_des = theta_des < 0 ? (2*M_PI + theta_des) : theta_des;
  next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;

  angle_error = theta_des - next_theta;
  angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
  angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

  //we say a maximum angle error would be off by 180 deg
  cost = ((M_PI/2.0) - angle_error)/(M_PI/2.0);
    
  return cost;
}


/************************************************************************
 *                   FIND THE OBSTACLE COST                             *
 *  This cost will be maximum if there is no obstacle along the         * 
 *  path of the velocity trajectory. The closer this trajectory         *
 *  is to an obstacle, the lower the cost will be                       *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindObstacleCost(VelocityStruct _velocity_struct, float _resolution) {

  bool collision = false;
  VelocityStruct velocity_struct;
  float cost, dist_to_collision, time_to_impact, sim_time, resolution;

  velocity_struct = _velocity_struct;
  resolution = _resolution;

  //sim_time is time needed to go through 90 deg
  sim_time = (M_PI/4) / velocity_struct.rot_vel;
  
  collision = collisionObject->Task(sim_time, resolution, velocity_struct.trans_vel, velocity_struct.rot_vel, time_to_impact);
  if(!collision) return 1.0;
  float radius = velocity_struct.trans_vel/velocity_struct.rot_vel;
  float circumference = 2*M_PI*radius;
  dist_to_collision = circumference*velocity_struct.rot_vel*time_to_impact/(2*M_PI);

  //I'm only going to check collision until a quarter of the circle has been travelled
  cost = dist_to_collision/(0.25*circumference);
    
  return cost;
}


/************************************************************************
 *                   FIND THE VELOCITY COST                             *
 *  The higher the translational velocity the higher this cost          * 
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindVelocityCost(VelocityStruct _velocity_struct) {

  VelocityStruct velocity_struct = _velocity_struct;
  float cost;

  cost = velocity_struct.trans_vel/max_trans_vel; //scaled between 0 and 1
  
  return cost;
}

/************************************************************************
 *                        ODOM CALLBACK                                 *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::OdomCallback(nav_msgs::Odometry msg) {
  odom_msg = msg;
  received_odom = true;
}

/************************************************************************
 *                        PUBLISH VELOCITY                              *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::PublishVel(float trans_vel, float rot_vel) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = trans_vel;
  cmd_vel.angular.z = rot_vel;
  vel_pub.publish(cmd_vel);
}


/************************************************************************
 *                        CHECK IF REACHED GOAL                         *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
bool DWA::ReachedGoal(float _goal_x, float _goal_y) {
  
  float threshold, goal_x, goal_y, dist_to_goal;
  float curr_x, curr_y;
  
  goal_x = _goal_x;
  goal_y = _goal_y;
  threshold = 0.1; //stop when 0.1m from goal
  curr_x = odom_msg.pose.pose.position.x;
  curr_y = odom_msg.pose.pose.position.y;
  
  dist_to_goal = sqrt(pow((curr_x - goal_x), 2) + pow((curr_y - goal_y), 2));

  if(dist_to_goal <= threshold) return true;
  return false;
}
