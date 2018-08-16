/*
 *
 *
 *
 *
 *
 */

#include "sim_path.h"


/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
Path::Path() : AperiodicTask() {
  path_pub = nh.advertise<testing::Path_msg>("vehicle_path", 1);
  odom_sub = nh.subscribe("odom", 1, &Path::OdomCB, this);
}


/***********************************************************************
 *                                                                     *
 *                      INIT FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int Path::Init() {
  received_odom = false;
  path_msg.reached_end = false;
  path_msg.event_point = false;
  
  return AperiodicTask::Init((char *) "path_task", 50);
}

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
void Path::Task() {
  while(!received_odom && ros::ok()) {
    ros::spinOnce();
  }

  float odom_x, odom_y, tracking_distance, curr_max_vel, event_duration;
  double des_northing, des_easting;
  int event_id, marker_id;
  std::ifstream inFile, velFile, eventFile;
  std::string str_northing, str_easting, str_vel, event_str;
  std::vector<double> northings;
  std::vector<double> eastings;
  std::vector<float> velocities;
  int vector_size = 2; 
  int line_count = 0;
  
  //open path and velocity file
  char *pEnd;
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/sim_path.txt");
  if (!inFile) {
    ROS_WARN("unable to open path file");
  }

  velFile.open("/home/robot/catkin_ws/src/testing/gps_files/sim_vel.txt");
  if (!velFile) {
    ROS_WARN("unable to open vel file");
  }

  eventFile.open("/home/robot/catkin_ws/src/testing/gps_files/sim_event_file.txt");
  if (!eventFile) {
    ROS_WARN("unable to open event file");
  }

  odom_x = odom_msg.pose.pose.position.x;
  odom_y = odom_msg.pose.pose.position.y;
    
  northings.push_back(odom_x);
  eastings.push_back(odom_y);
  velocities.push_back(0.5);

  std::getline(inFile, str_northing);
  std::getline(inFile, str_easting);
  std::getline(velFile, str_vel);
  northings.insert(northings.begin(), strtof(str_northing.c_str(), &pEnd));
  eastings.insert(eastings.begin(), strtof(str_easting.c_str(), &pEnd));
  velocities.push_back(strtof(str_vel.c_str(), &pEnd));

  des_northing = northings[1];
  des_easting = eastings[1];

  bool update_event_point = true;
  
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    odom_x = odom_msg.pose.pose.position.x;
    odom_y = odom_msg.pose.pose.position.y;

    //event point
    //if(update_event_point) {
    if(false) {
      int index1, index2;
      std::getline(eventFile, event_str);
      while(event_str.substr(0, 5) != "event") std::getline(eventFile, event_str);
      
      index1 = event_str.find_last_of(":");
      index2 = event_str.size() - 1;
      event_str = event_str.substr(index1 + 2, index2);
      event_id = stoi(event_str);

      std::getline(eventFile, event_str);
      index1 = event_str.find_last_of(":");
      index2 = event_str.size() - 1;
      event_str = event_str.substr(index1 + 2, index2);
      marker_id = stoi(event_str);

      std::getline(eventFile, event_str);
      index1 = event_str.find_last_of(":");
      index2 = event_str.size() - 1;
      event_str = event_str.substr(index1 + 2, index2);
      event_duration = strtof(event_str.c_str(), &pEnd);
	
      update_event_point = false;
    }
    
    tracking_distance = FindLookAheadDistance(); // Find the look ahead distance

    //project the position of the car onto the path
    double x1, x2, y1, y2, b, c, a, x_line, y_line, dist1, dist2;
    x1 = northings[vector_size - 1]; // lat/lon marker behind you
    y1 = eastings[vector_size - 1];
    x2 = northings[vector_size - 2]; //lat/lon marker ahead of you
    y2 = eastings[vector_size - 2];
    a = (y2 - y1); // line between these two markers
    b = -(x2 - x1);
    c = x2*y1 - x1*y2;
    x_line = (b*(b*odom_x - a*odom_y) - a*c)/(pow(a, 2) + pow(b, 2)); // where the car is in this line
    y_line = (a*(-b*odom_x + a*odom_y) - b*c)/(pow(a, 2) + pow(b, 2));
    dist2 = sqrt(pow((x_line - x2), 2) + pow((y_line - y2), 2)); // dist from car to next lat/lon marker
    dist1 = sqrt(pow((x_line - x1), 2) + pow((y_line - y1), 2));
    
    // if the look ahead distance is before the next lat lon marker stay looking at that line
    if(tracking_distance <= dist2 || inFile.eof()) {
      des_northing = x_line + tracking_distance*(x2 - x_line)/dist2;
      des_easting = y_line + tracking_distance*(y2 - y_line)/dist2;
    }
    else { //else read in new points and look through those new lines
      double path_dist, dist_left, line_dist;
      int iterator;
      path_dist = dist2;
      iterator = 3;
    
      while(true) {
	if(vector_size < iterator){
	  std::getline(inFile, str_northing);
	  if(inFile.eof()) {
	    break;
	  }
	  std::getline(inFile, str_easting);
	  northings.insert(northings.begin(), strtof(str_northing.c_str(), &pEnd));
	  eastings.insert(eastings.begin(), strtof(str_easting.c_str(), &pEnd));
	  vector_size += 1;
	}
	path_dist += sqrt(pow(eastings[vector_size - iterator] - eastings[vector_size - iterator + 1], 2)\
			  + pow(northings[vector_size - iterator] - northings[vector_size - iterator + 1], 2));
	iterator += 1;
	if(path_dist > tracking_distance) {break;}
	
      }
      if(!inFile.eof()) {
	dist_left = dist2;
	for (int i = 0; i < (vector_size - 3); i++) {
	  dist_left += sqrt(pow(eastings[vector_size - 2 - i] - eastings[vector_size - 3 - i], 2)\
			    + pow(northings[vector_size - 2 - i] - northings[vector_size - 3 - i], 2));
	}
	dist_left = tracking_distance - dist_left;
	line_dist = sqrt(pow(eastings[0] - eastings[1], 2) + pow(northings[0] - northings[1], 2));
	des_northing = northings[1] + dist_left*(northings[0] - northings[1])/line_dist;
	des_easting = eastings[1] + dist_left*(eastings[0] - eastings[1])/line_dist;
      }      
    }

    //Find the current maximum velocity
    curr_max_vel = FindCurrMaxVel(velocities[1], velocities[0], dist2, dist1); 
    
    //Update which line you are on
    if(vector_size > 2) {
      float dist_to_segment1 = FindDistToSegment(northings[vector_size - 2], eastings[vector_size - 2], northings[vector_size - 1], eastings[vector_size - 1]);
      float dist_to_segment2 = FindDistToSegment(northings[vector_size - 3], eastings[vector_size - 3], northings[vector_size - 2], eastings[vector_size - 2]);
      if(dist_to_segment1 > dist_to_segment2 /*|| dist2 < 1.0*/) {
	northings.pop_back();
	eastings.pop_back();
	velocities.pop_back();
	std::getline(velFile, str_vel);
	velocities.push_back(strtof(str_vel.c_str(), &pEnd));
	vector_size -= 1;
	ROS_INFO("Switched paths!\n");

	line_count += 1;
	if(line_count == marker_id) {
	  path_msg.event_duration = event_duration;
	  path_msg.event_id = event_id;
	  path_msg.event_point = true;
	  update_event_point = true;
	}
	else path_msg.event_point = false;
      }
    }

    if(inFile.eof()) {
      if(dist2 < 1.0) {
	ROS_INFO("REACHED END OF PATH\n");
	path_msg.reached_end = true;
      }
    }

    path_msg.northing1 = northings[vector_size - 1];
    path_msg.easting1 = eastings[vector_size - 1];
    path_msg.northing2 = northings[vector_size - 2];
    path_msg.easting2 = eastings[vector_size - 2];
    path_msg.des_northing = des_northing;
    path_msg.des_easting = des_easting;
    path_msg.max_vel = curr_max_vel;

    // path_msg.northing1 = 0.0;
    // path_msg.easting1 = 0.0;
    // path_msg.northing2 = 5.0;
    // path_msg.easting2 = 0.0;
    // path_msg.des_northing = 0.0;
    // path_msg.des_easting = -3.0;
    // path_msg.max_vel = 0.5;

    
    path_pub.publish(path_msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  inFile.close();
}


/***********************************************************************
 *                                                                     *
 *          FIND THE DISTANCE THE VEHICLE IS LOOKING AHEAD             *
 *                                                                     *
 *********************************************************************/
float Path::FindLookAheadDistance() {
  float curr_vel = odom_msg.twist.twist.linear.x;
  float min_vel = 0.5;
  float max_vel = 0.75;
  float lower_limit = min_vel + 0.1*min_vel;
  float upper_limit = max_vel - 0.1*max_vel;

  float min_look_ahead = 2.0;
  float max_look_ahead = 5.0;
  
  if(curr_vel < lower_limit) return min_look_ahead; //2 meters
  else if(curr_vel > upper_limit) return max_look_ahead; //6 meters?
  else {
    return (min_look_ahead + (max_look_ahead - min_look_ahead)*(curr_vel - min_vel)/(max_vel - min_vel));
  }
}

/***********************************************************************
 *                                                                     *
 *               FIND CARS CURRENT MAX VELOCITY                        *
 *                                                                     *
 *********************************************************************/
float Path::FindCurrMaxVel(float _vel2, float _vel1, float _dist2, float _dist1) {
  float vel1, vel2, dist1, dist2;
  vel2 = _vel2;
  vel1 = _vel1;
  dist2 = _dist2;
  dist1 = _dist1;

  float rate = 0.1; //increase in velocity per meter you move away from corners
  float abs_max_vel = 0.75; //dont go higher than 0.75 m/s for now
  float abs_min_vel = 0.5;
  float vel_dist_1 = vel1 + rate*dist1;
  float vel_dist_2 = vel2 + rate*dist2;

  float final_vel = (vel_dist_1 <= vel_dist_2)*vel_dist_1 + (vel_dist_2 < vel_dist_1)*vel_dist_2;
  if(final_vel > abs_max_vel) return abs_max_vel;
  if(final_vel < abs_min_vel) return abs_min_vel;
  return final_vel;
}


/***********************************************************************
 *                                                                     *
 *               FIND DISTANCE FROM CAR TO LINE SEGMENT                *
 *                                                                     *
 *********************************************************************/
double Path::FindDistToSegment(double _x1, double _y1, double _x2, double _y2) {
  double x1, x2, y1, y2, px, py, u, x, y, dx, dy;
  float odom_x = odom_msg.pose.pose.position.x;
  float odom_y = odom_msg.pose.pose.position.y;

  x1 = _x1;
  x2 = _x2;
  y1 = _y1;
  y2 = _y2;

  px = x2 - x1;
  py = y2 - y1;

  u = ((odom_x - x1)*px + (odom_y - y1)*py)/(pow(px, 2) + pow(py, 2));

  if (u > 1.0) u = 1.0;
  else if (u < 0.0) u = 0.0;

  x = x1 + u*px;
  y = y1 + u*py;

  dx = x - odom_x;
  dy = y - odom_y;

  return sqrt(pow(dx, 2) + pow(dy, 2));
}

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void Path::OdomCB(nav_msgs::Odometry msg) {
  received_odom = true;
  odom_msg = msg;
}
