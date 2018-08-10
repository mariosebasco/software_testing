/*
 *
 *
 *
 */


#include "A_star.h"

/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
A_star::A_star() : AperiodicTask() {
  VEHICLE_WIDTH = 0.4;
  
  odom_sub = nh.subscribe("odom", 1, &A_star::OdomCB, this);
  //odom_sub = nh.subscribe("local/odom", 1, &A_star::OdomCB, this);
  costmap_sub = nh.subscribe("costmap_node/costmap/costmap", 1, &A_star::CostmapCB, this);
  path_pub = nh.advertise<nav_msgs::Path>("A_star_path", 1);
}

/***********************************************************************
 *                                                                     *
 *                          INIT FUNCTION                              *
 *                                                                     *
 *********************************************************************/
int A_star::Init() {
  received_odom = false;
  received_costmap = false;
  INTERRUPT = false;
  
  return AperiodicTask::Init((char*) "a_star", 20);
}


/***********************************************************************
 *                                                                     *
 *                          MAIN FUNCTION                              *
 *                                                                     *
 *********************************************************************/
void A_star::Task() {

  while(!received_odom && !received_costmap) {
    ros::spinOnce();
  }

  //convert goal to integer value for A* algorithm
  float map_resolution = costmap.info.resolution;
  GOAL_X_LOCAL = 5;
  GOAL_Y_LOCAL = 0;
  GOAL_X = int(GOAL_X_LOCAL/map_resolution);
  GOAL_Y = int(GOAL_Y_LOCAL/map_resolution);

  printf("goal x: %d\n", GOAL_X);  
  printf("goal y: %d\n", GOAL_Y);
  
  std::set<std::string> frontier;
  std::set<std::string> explored;
  std::unordered_map<std::string, int> dist_map;
  std::unordered_map<std::string, std::string> parent_map;

  ros::Rate loop_rate(1);
  while(ros::ok() && !INTERRUPT) {
    bool found_path = false;
    bool node_added;
    float best_cost, temp_cost, best_dist, temp_dist;
    float max_cost = 1000000.0;
    std::set<std::string>::iterator it;
    int x_val, y_val, str_size, under_score, start_x, start_y;
    std::string best_name, temp_name;

    x_val = 0;
    y_val = 0;
    start_x = 0;
    start_y = 0;
    
    frontier.clear();
    explored.clear();
    dist_map.clear();
    parent_map.clear();
    
    temp_name = std::to_string(start_x) + "_" + std::to_string(start_y);
    frontier.insert(temp_name);
    dist_map[temp_name] = 0;

    while(ros::ok() && !found_path && !INTERRUPT) {
      best_cost = max_cost;

      //iterate through frontier to decide which node to expand
      for(it = frontier.begin(); it != frontier.end(); ++it) {
	str_size = (*it).size();
	under_score = (*it).find("_");
	x_val = std::stoi((*it).substr(0, under_score));
	y_val = std::stoi((*it).substr(under_score + 1, str_size - 1));
	temp_dist = dist_map[(*it)];
	temp_cost = temp_dist + sqrt(pow(x_val - GOAL_X, 2) + pow(y_val - GOAL_Y, 2));
	if(temp_cost < best_cost) {
	  best_cost = temp_cost;
	  best_dist = temp_dist;
	  best_name = (*it);
	}
      }

      if(best_cost == max_cost) {//nothing on the frontier?
	ROS_WARN("A* ALGORITHM FAILED TO FIND PATH");
	return;
      }

      for(int dx = -1; dx < 2; dx++) {
	if(found_path) break;
	for(int dy = -1; dy < 2; dy++) {
	  if(dx == 0 && dy == 0) continue;
	  str_size = best_name.size();
	  under_score = best_name.find("_");
	  x_val = std::stoi(best_name.substr(0, under_score));
	  y_val = std::stoi(best_name.substr(under_score + 1, str_size - 1));
	  x_val += dx;
	  y_val += dy;
	  temp_dist = dist_map[best_name] + sqrt(dx*dx + dy*dy);

	  //check to see if goal node
	  if(x_val == GOAL_X && y_val == GOAL_Y) {
	    found_path = true;
	    temp_name = std::to_string(x_val) + "_" + std::to_string(y_val);
	    frontier.insert(temp_name);
	    dist_map[temp_name] = temp_dist;
	    parent_map[temp_name] = best_name;
	    break;
	  }
	  
	  //check to see if already in frontier or explored sets
	  temp_name = std::to_string(x_val) + "_" + std::to_string(y_val);
	  if(frontier.find(temp_name) != frontier.end()) continue;
	  if(explored.find(temp_name) != explored.end()) continue;

	  //check to see if location has collision
	  if (CheckCostmap(x_val, y_val)) continue;

	  //if none of the above, add node to frontier
	  frontier.insert(temp_name);
	  dist_map[temp_name] = temp_dist;
	  parent_map[temp_name] = best_name;
	}
      }

      //remove the expanded node from the frontier
      if(!found_path) {
	frontier.erase(best_name);
	explored.insert(best_name);
      }
    }

    //if out here then solution has been found
    float actual_x, actual_y;
    nav_msgs::Path final_path;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::PoseStamped> pose_vector;
    
    std::string curr_name = std::to_string(x_val) + "_" + std::to_string(y_val);
    std::string start_name = std::to_string(start_x) + "_" + std::to_string(start_y);
    
    final_path.header.stamp = ros::Time::now();
    final_path.header.frame_id = "odom";

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "odom";

    printf("before while loop\n");
    while(curr_name != start_name) {
      str_size = curr_name.size();
      under_score = (curr_name).find("_");
      x_val = std::stoi((curr_name).substr(0, under_score));
      y_val = std::stoi((curr_name).substr(under_score + 1, str_size - 1));
      actual_x = x_val*map_resolution + odom_msg.pose.pose.position.x;
      actual_y = y_val*map_resolution + odom_msg.pose.pose.position.y;

      // printf("x_val: %d\n", x_val);
      // printf("y_val: %d\n", y_val);
      
      pose.pose.position.x = actual_x;
      pose.pose.position.y = actual_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      final_path.poses.insert(final_path.poses.begin(), pose);
      //std::cout << curr_name << std::endl;

      // for(it = frontier.begin(); it != frontier.end(); it++) {
      // 	std::cout << *it << std::endl;
      // 	std::cout << parent_map[*it] << std::endl;
      // }
      
      curr_name = parent_map[curr_name];
      //std::cout << curr_name << std::endl;
      
    }
    
    //final_path->poses = poses;
    path_pub.publish(final_path);

    ros::spinOnce();
    loop_rate.sleep();
  }
}



/***********************************************************************
 *                                                                     *
 *                          ODOM CALLBACK                              *
 *                                                                     *
 *********************************************************************/
void A_star::OdomCB(const nav_msgs::Odometry &msg) {
  received_odom = true;
  odom_msg = msg;
}

/***********************************************************************
 *                                                                     *
 *                          COSTMAP CALLBACK                           *
 *                                                                     *
 *********************************************************************/
void A_star::CostmapCB(const nav_msgs::OccupancyGrid &msg) {
  received_costmap = true;
  costmap = msg;
}

/***********************************************************************
 *                                                                     *
 *                   CHECK IF COSTMAP IS OCCUPIED                      *
 *                                                                     *
 *********************************************************************/
bool A_star::CheckCostmap(int _x, int _y) {
  double origin_x, origin_y;
  float map_resolution;
  int map_width, map_height, cell_value, grid_cell_x, grid_cell_y, grid_cell;
    
  origin_x = costmap.info.origin.position.x;
  origin_y = costmap.info.origin.position.y;
  map_resolution = costmap.info.resolution;
  map_height = costmap.info.height;
  map_width = costmap.info.width;

  //if outside of costmap just say there's no collision for now  
  if(abs(_x) > map_width/2) return false;
  if(abs(_y) > map_height/2) return false;
  
  float x_pos = map_resolution*_x;
  float y_pos = map_resolution*_y;
  
  grid_cell_y = int(fabs(origin_y + y_pos)/map_resolution)*map_width;
  grid_cell_x = int(fabs(origin_x - x_pos)/map_resolution); 
  grid_cell = grid_cell_y + grid_cell_x;

  cell_value = costmap.data[grid_cell];

  if(cell_value > 0) return true;

  //let's check around the point -- we want it to be clear of obtacles
  //we're going to check a cricle around it of diameter = vehicle_width
  //check every 2 degrees to see if the grid is occupied
  float tot_degrees = 2.0*M_PI;
  float resolution = 5.0*M_PI/180.0;
  int num_iterations = int(tot_degrees/resolution);
  float radius = 0.3;//VEHICLE_WIDTH/2.0;
  float curr_x, curr_y;
  
  for (int i = 0; i < num_iterations; i++) {
    curr_x = x_pos + radius*cos(i*resolution);
    curr_y = y_pos + radius*sin(i*resolution);

    grid_cell_y = int(fabs(origin_y + curr_y)/map_resolution)*map_width;
    grid_cell_x = int(fabs(origin_x - curr_x)/map_resolution); 
    grid_cell= grid_cell_y + grid_cell_x;

    cell_value = costmap.data[grid_cell];
    if(cell_value > 0) return true;
  }

  return false;
}

