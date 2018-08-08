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
  odom_sub_global = nh.subscribe("odom", 1, &DWA::OdomGlobalCB, this);
  path_sub = nh.subscribe("vehicle_path", 1, &DWA::PathCB, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
  // GOAL_X = 4.0;
  // GOAL_Y = 0.0;
  // ORIENTATION = 0.0;

  MAX_TRANS_VEL = 0.40; //0.25m/s for testing
  MAX_ROT_VEL = 40.0*M_PI / 180.0;//M_PI/4.0;
  MAX_TRANS_ACCELERATION = 0.5;
  MAX_ROT_ACCELERATION = 1.57;
  SIM_TIME = 3.0;
  RESOLUTION = 0.1;
  MAX_DIST_FROM_PATH = 2.0; //don't deviate from path by more than 2 meters
  // nh.getParam("/max_trans_acceleration", max_trans_acceleration);
  // nh.getParam("/max_rot_acceleration", max_rot_acceleration);
  // nh.getParam("/sim_time_dwa", sim_time);
  // nh.getParam("/resolution", resolution);
  
}

int DWA::Init() {
  received_odom = false;
  received_odom_global = false;
  received_path = false;
  received_a_star = false;
  INTERRUPT = false;
  
  return AperiodicTask::Init((char *) "localTrackTask", 50);
}


void DWA::Task() {

  //make sure you've received odometry and path data
  while(!received_odom && !received_path && !received_odom_global) {
    ros::spinOnce();
  }

  float goal_x, goal_y, curr_x, curr_y;
  float orientation_cost, obstacle_cost, velocity_cost, path_cost, distance_cost, a_star_cost;
  float alpha, beta, gamma, delta, zeta;
  float optimal_trans_vel, optimal_rot_vel;
  float max_path_cost = 0.0;

  alpha = 0.8; //orientation
  beta = 0.8; //obstacle
  gamma = 0.3; //velocity
  zeta = 0.3; //distance
  
  VelocityStruct velocity_struct;
  float opt_obstacle, opt_orientation, opt_vel, opt_distance, dist_to_goal;
  
  ros::Rate loop_rate(10.0);
    
  while(ros::ok() && !INTERRUPT) {

    curr_x = odom_msg.pose.pose.position.x;
    curr_y = odom_msg.pose.pose.position.y;
    goal_x = path_msg.des_northing - odom_msg_global.pose.pose.position.x + curr_x;
    goal_y = path_msg.des_easting - odom_msg_global.pose.pose.position.y + curr_y;
    if(path_msg.reached_end) break;
    
    //find the set of possible velocities (Dynamic Window)
    float curr_vel = odom_msg.twist.twist.linear.x;
    float curr_rot_vel = odom_msg.twist.twist.angular.z;
    float vel_upper_limit = curr_vel + MAX_TRANS_ACCELERATION*SIM_TIME;
    float vel_lower_limit = curr_vel - MAX_TRANS_ACCELERATION*SIM_TIME;
    float rot_upper_limit = curr_rot_vel + MAX_ROT_ACCELERATION*SIM_TIME;
    float rot_lower_limit = curr_rot_vel - MAX_ROT_ACCELERATION*SIM_TIME;
  
    if(vel_upper_limit > MAX_TRANS_VEL) vel_upper_limit = MAX_TRANS_VEL;
    if(vel_lower_limit < 0.1) vel_lower_limit = 0.1;
    if(rot_upper_limit > MAX_ROT_VEL) rot_upper_limit = MAX_ROT_VEL;
    if(rot_lower_limit < -MAX_ROT_VEL) rot_lower_limit = -MAX_ROT_VEL;

    float curr_dist_to_goal = sqrt(pow((curr_x - goal_x), 2) + pow((curr_y - goal_y), 2));
    
    float num_trans_iterations = int((vel_upper_limit - vel_lower_limit)/RESOLUTION);
    float num_rot_iterations = int((rot_upper_limit - rot_lower_limit)/RESOLUTION);

    float temp_trans_vel = 0.0;
    float temp_rot_vel = 0.0;

    double x1 = path_msg.northing1;
    double y1 = path_msg.easting1;
    double x2 = path_msg.northing1;
    double y2 = path_msg.easting1;
    
    float dist_from_path = FindDistFromPath(x1, y1, x2, y2);
    if(dist_from_path > MAX_DIST_FROM_PATH) {
      PublishVel(0.0, 0.0);
      ROS_WARN("STOPPING -- REACHED MAX DISTANCE FROM PATH\n");
      return;
    }
    
    //update odom and costmap for the collision class
    collisionObject->UpdateCallbacks();
    
    for(int i = 0; i <= num_trans_iterations; i++) {
      temp_trans_vel = vel_lower_limit + i*RESOLUTION;
      
      for(int j = 0; j <= num_rot_iterations; j++) {

    	temp_rot_vel = rot_lower_limit + j*RESOLUTION;
    	velocity_struct.trans_vel = temp_trans_vel;
    	velocity_struct.rot_vel = temp_rot_vel;

    	obstacle_cost = FindObstacleCost(velocity_struct);
    	if(obstacle_cost == 0.0) {continue;}
    	orientation_cost = FindOrientationCost(velocity_struct, goal_x, goal_y, dist_to_goal);
	if(orientation_cost == 0.0) {continue;}
    	velocity_cost = FindVelocityCost(velocity_struct);
	if(dist_to_goal > curr_dist_to_goal) distance_cost = 0.0;
	else distance_cost = (curr_dist_to_goal - dist_to_goal);
	if(received_a_star) a_star_cost = FindPathCost(velocity_struct);
	else a_star_cost = 0.0;
    	path_cost = alpha*orientation_cost\
	  + beta*obstacle_cost\
	  + gamma*velocity_cost\
	  + delta*a_star_cost\
	  + zeta*distance_cost;

	//**********************************************************************
	// TESTING
	// printf("lin vel: %f\n", temp_trans_vel);
	// printf("rot vel: %f\n", temp_rot_vel);
	  
	// printf("orientation cost: %f\n", orientation_cost);
	// printf("obstable cost: %f\n", obstacle_cost);
	// printf("velocity cost: %f\n", velocity_cost);
	// printf("distance cost: %f\n", distance_cost);
	// printf("total cost: %f\n", path_cost);
	// std::cout << "****************************" <<std::endl << std::endl;

	// char myChar;
	// std::cin >> myChar;
	// collisionObject->UpdateCallbacks();
	
	//**********************************************************************
	
    	//update optimal velocity
    	if(path_cost > max_path_cost) {
    	  max_path_cost = path_cost;
    	  optimal_trans_vel = velocity_struct.trans_vel;
    	  optimal_rot_vel = velocity_struct.rot_vel;
    	  opt_obstacle = obstacle_cost;
    	  opt_orientation = orientation_cost;
    	  opt_vel = velocity_cost;
    	  opt_distance = distance_cost;

    	}
      }
    }

    PublishVel(optimal_trans_vel, optimal_rot_vel);
    
    max_path_cost = 0.0;
    optimal_trans_vel = 0.0;
    optimal_rot_vel = 0.0;
          
    ros::spinOnce();
    loop_rate.sleep();
  }

  //if out here you've reached goal
  PublishVel(0.0,0.0);
}


/************************************************************************
 *                  FIND THE ORIENTATION COST                           *
 *  We want to favor velocities whose final orientation will face       * 
 *  the goal. Cost = 1 is directly facing the goal                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindOrientationCost(VelocityStruct _velocity_struct, float _goal_x, float _goal_y, float &dist_to_goal) {

  VelocityStruct velocity_struct;
  float cost, goal_x, goal_y, radius, circle_x, circle_y;
  float curr_x, curr_y, curr_theta, next_x, next_y, next_theta;
  
  velocity_struct = _velocity_struct;
  goal_x = _goal_x;
  goal_y = _goal_y;

  curr_theta = getYaw(odom_quat);
  curr_theta = curr_theta < 0 ? (2*M_PI + curr_theta) : curr_theta;
  curr_x = odom_msg.pose.pose.position.x;
  curr_y = odom_msg.pose.pose.position.y;
  
  if(velocity_struct.rot_vel == 0.0) {
    next_theta = curr_theta;
    next_x = curr_x + cos(curr_theta)*velocity_struct.trans_vel*SIM_TIME;
    next_y = curr_y + sin(curr_theta)*velocity_struct.trans_vel*SIM_TIME;
  }
  else {
    radius = velocity_struct.trans_vel/velocity_struct.rot_vel;
    circle_x = curr_x - radius*sin(curr_theta);
    circle_y = curr_y + radius*cos(curr_theta);

    next_theta = curr_theta + velocity_struct.rot_vel*SIM_TIME;
    next_x = circle_x + radius*sin(next_theta);
    next_y = circle_y - radius*cos(next_theta);
  }
  
  dist_to_goal = sqrt(pow((next_x - goal_x), 2) + pow((next_y - goal_y), 2));
  
  next_theta = next_theta > 2*M_PI ? (next_theta - 2*M_PI) : next_theta;
  float theta_des, theta_curr, angle_error;
  theta_des = atan2((goal_y - next_y), (goal_x - next_x));
  theta_des = theta_des < 0 ? (2*M_PI + theta_des) : theta_des;
  next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
  
  angle_error = theta_des - next_theta;
  angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
  angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

  //we say a maximum angle error would be off by 180 deg
  cost = (M_PI - fabs(angle_error))/M_PI;
  return cost;
}


/************************************************************************
 *                   FIND THE OBSTACLE COST                             *
 *  This cost will be maximum if there is no obstacle along the         * 
 *  path of the velocity trajectory. The closer this trajectory         *
 *  is to an obstacle, the lower the cost will be                       *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindObstacleCost(VelocityStruct _velocity_struct) {

  bool collision = false;
  VelocityStruct velocity_struct;
  float cost, dist_to_collision, time_to_impact, sim_time, resolution;

  velocity_struct = _velocity_struct;

  if(velocity_struct.trans_vel == 0.0) {
  //if(true) {
    sim_time = 10.0;
    resolution = 1.0;
  }
  else {
    float dist_forward = 3.0;
    sim_time = dist_forward/velocity_struct.trans_vel;
    if(sim_time > 20.0) sim_time = 20;
    resolution = sim_time / (dist_forward*4.0); //check collision 4 times every meter
  }

  
  collision = collisionObject->Task(sim_time, resolution, velocity_struct.trans_vel, velocity_struct.rot_vel, time_to_impact);
  if(!collision) return 1.0;
  if((time_to_impact <= SIM_TIME)) return 0.0;
  else cost = time_to_impact / sim_time;

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

  cost = velocity_struct.trans_vel/MAX_TRANS_VEL; //scaled between 0 and 1
  
  return cost;
}

float DWA::FindPathCost(VelocityStruct _velocity_struct) {
  
  
  return 0.0;
}


/************************************************************************
 *                   CHECK CURRENT DISTANCE FROM PATH                   *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
float DWA::FindDistFromPath(float _x1, float _y1, float _x2, float _y2) {
  float x1, x2, y1, y2, a, b, c, dist;
  float odom_x = odom_msg.pose.pose.position.x;
  float odom_y = odom_msg.pose.pose.position.y;

  x1 = _x1;
  y1 = _y1;
  x2 = _x2;
  y2 = _y2;
  
  a = (y2 - y1); // line between these two markers
  b = -(x2 - x1);
  c = x2*y1 - x1*y2;

  dist = fabs(a*odom_x + b*odom_y + c)/sqrt(pow(a, 2) + pow(b, 2));
  return dist;
}


/************************************************************************
 *                        ODOM CALLBACK                                 *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::OdomCallback(nav_msgs::Odometry msg) {
  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  received_odom = true;
}

/************************************************************************
 *                 GLOBAL ODOM CALLBACK                                 *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::OdomGlobalCB(nav_msgs::Odometry msg) {
  odom_msg_global = msg;
  received_odom_global = true;
}


/************************************************************************
 *                        PATH CALLBACK                                 *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::PathCB(testing::Path_msg msg) {
  path_msg = msg;
  received_path = true;
}


/************************************************************************
 *                             A STAR CALLBACK                          *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void DWA::AStarCB(nav_msgs::Path msg) {
  received_a_star = true;
  a_star_path = msg;
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
