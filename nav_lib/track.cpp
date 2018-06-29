/*
 *
 *  Path follower - 
 *  INPUT: Path
 *  OUTPUT: command velocity
 *
 */


#include "track.h"

/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
TrackPoint::TrackPoint(Collision* _collisionObject) : AperiodicTask() {
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odom_sub = nh.subscribe("odom", 1, &TrackPoint::OdomCallback, this);    
  should_start = false;

  collisionObject = _collisionObject;
  
  position_KP = 1.0;
  angle_KP = 1.0;
  turn_in_place_KP = 0.85;

  COLLISION_DETECTED = false;
}

/***********************************************************************
 *                                                                     *
 *                      INITIALIZATION                                 *
 *                                                                     *
 *********************************************************************/
int TrackPoint::Init() {

  return AperiodicTask::Init((char *) "trackTask", 40);
}


/***********************************************************************
 *                                                                     *
 *                             MAIN TASK                               *
 *                                                                     *
 *********************************************************************/
void TrackPoint::Task() {

  //wait for odom data before starting this function
  while(!should_start) {
    UpdateOdom();
  }
  
  float tracking_distance = 1.0; //how far down the path the point we are tracking is
  
  //import path file
  std::ifstream inFile, velFile;
  std::string str_northing, str_easting, str_vel;
  double des_northing, des_easting, point_dist;
  double curvature, ang_vel, lin_vel;
  float angle_error, curr_max_vel;
  bool finished_turning_in_place = false;
  std::vector<double> northings;
  std::vector<double> eastings;
  std::vector<float> velocities;
  int vector_size = 2; 

  float L, max_vel, sim_time, resolution;

  L = 0.4; //meters
  max_vel = 0.5; //m/s
  sim_time = 3; //seconds
  resolution = 0.25; //seconds
  curr_max_vel = 0.5;

  northings.push_back(odom_x);
  eastings.push_back(odom_y);
  velocities.push_back(0.5);

  //open path files and read first point, then turn to face it
  char *pEnd;
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");
  if (!inFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  velFile.open("/home/robot/catkin_ws/src/testing/gps_files/vel_map.txt");
  if (!velFile) {
    std::cout << "unable to open path file" << std::endl;
  }
  
  std::getline(inFile, str_northing);
  std::getline(inFile, str_easting);
  std::getline(velFile, str_vel);
  northings.push_back(strtof(str_northing.c_str(), &pEnd));
  eastings.push_back(strtof(str_easting.c_str(), &pEnd));
  velocities.push_back(strtof(str_vel.c_str(), &pEnd));

  des_northing = northings[1];
  des_easting = eastings[1];

  angle_error = FindAngleError(des_northing, des_easting);
  while(!finished_turning_in_place && ros::ok()) {
    //adjust angular velocity and publish speed
    //ang_vel = angle_error*turn_in_place_KP;
    if(angle_error > 0) PublishSpeed(0.0, 0.5);
    else PublishSpeed(0.0, -0.5);

    //update odometry
    UpdateOdom();

    //update error
    angle_error = FindAngleError(des_northing, des_easting);

    finished_turning_in_place = abs(angle_error*180/M_PI) < 8.0 ? true : false;
  }
  PublishSpeed(0.0, 0.0);
  
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    //update odometry
    UpdateOdom();

    //if collision is detected let the local planner take over and wait for it to finish
    if(collisionObject->Task(sim_time, resolution)) {
      printf("collision detected\n");
      GOAL_X = des_northing - odom_x;
      GOAL_Y = des_easting - odom_y;
      ORIENTATION = getYaw(odom_quat);

      COLLISION_DETECTED = true;
      PublishSpeed(0.0, 0.0);
      while(COLLISION_DETECTED) {
	ros::Duration(1.0).sleep();
      }
      UpdateOdom();
    }

    tracking_distance = FindLookAheadDistance(); // Find the look ahead distance

    //project the position of the car onto the path
    double x1, x2, y1, y2, b, c, a, x_line, y_line, dist1, dist2;
    float percent_travelled;
    x1 = northings[vector_size - 1]; // lat/lon marker behind you
    y1 = eastings[vector_size - 1];
    x2 = northings[vector_size - 2]; //lat/lon marker ahead of you
    y2 = eastings[vector_size - 2];
    a = (y2 - y1); // line between these two markers
    b = -(x2 - x1);
    c = x2*y1 - x1*y2;
    x_line = (b*(b*odom_x - a*odom_y) - a*c)/(pow(a, 2) + pow(b, 2)); // where the car is in this line
    y_line = (a*(-b*odom_x + a*odom_y) - b*c)/(pow(a, 2) + pow(b, 2));
    dist2 = sqrt(pow((x_line - x2), 2) + pow((y_line - y2), 2)); // dist form car to next lat/lon marker
    dist1 = sqrt(pow((x_line - x1), 2) + pow((y_line - y1), 2)); 

    // if the look ahead distance is before the next lat lon marker stay looking at that line
    if(dist2 <= tracking_distance) { 
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
	  std::getline(inFile, str_easting);
	  northings.push_back(strtof(str_northing.c_str(), &pEnd));
	  eastings.push_back(strtof(str_easting.c_str(), &pEnd));
	  vector_size += 1;
	}
	path_dist += sqrt(pow(eastings[vector_size - iterator] - eastings[vector_size - iterator + 1], 2)\
			  + pow(northings[vector_size - iterator] - northings[vector_size - iterator + 1], 2));
	iterator += 1;
	if(path_dist > tracking_distance) {break;}
      }
      dist_left = dist2;
      for (int i = 0; i < (vector_size - 3); i++) {
	dist_left += sqrt(pow(eastings[vector_size - 2 - i] - eastings[vector_size - 3 - i], 2)\
			  + pow(northings[vector_size - 2 - i] - northings[vector_size - 3 - i], 2));
      }
      dist_left = tracking_distance - dist_left;
      line_dist = sqrt(pow(eastings[0] - eastings[1], 2) + pow(northings[0] - northings[1], 2));
      des_northing = northings[1] + dist_left*(northings[1] - northings[0])/line_dist;
      des_easting = eastings[1] + dist_left*(eastings[1] - eastings[0])/line_dist;
    }

    //Find the current maximum velocity
    curr_max_vel = FindCurrMaxVel(velocities[1], velocities[0], dist2, dist1); 

    //if car is less than one meter from lat/lon marker say we are now in the next line(path)
    if(dist2 > 1.0) {
      northings.pop_back();
      eastings.pop_back();
      velocities.pop_back();
      std::getline(velFile, str_vel);
      velocities.push_back(strtof(str_vel.c_str(), &pEnd));
      vector_size -= 1;
      
      x1 = northings[vector_size - 1];
      y1 = eastings[vector_size - 1];
      x2 = northings[vector_size - 2];
      y2 = eastings[vector_size - 2];
      a = (y2 - y1);
      b = -(x2 - x1);
      c = x2*y1 - x1*y2;
      x_line = (b*(b*odom_x - a*odom_y) - a*c)/(pow(a, 2) + pow(b, 2));
      y_line = (a*(-b*odom_x + a*odom_y) - b*c)/(pow(a, 2) + pow(b, 2));
      dist2 = sqrt(pow((x_line - x2), 2) + pow((y_line - y2), 2));
      dist1 = sqrt(pow((x_line - x1), 2) + pow((y_line - y1), 2)); 
    }

    //now we can start sending command velocities
    angle_error = FindAngleError(des_northing, des_easting);

    //if you are facing the wrong direction turn around
    if(abs(angle_error) > (70.0*M_PI/180.0)) {
      while(!finished_turning_in_place && ros::ok()) {

    	//adjust angular velocity and publish speed
    	//ang_vel = angle_error*turn_in_place_KP;
	if(angle_error > 0) PublishSpeed(0.0, 0.5);
	else PublishSpeed(0.0, -0.5);

    	//update odometry
    	UpdateOdom();

    	//update error
    	angle_error = FindAngleError(des_northing, des_easting);

    	finished_turning_in_place = abs(angle_error*180/M_PI) < 8.0 ? true : false;
      }
      PublishSpeed(0.0, 0.0);
    }
    
    //find the curvature given the point - 1/R = 2x/D^2
    float theta_curr = getYaw(odom_quat);// + imu_drift;
    float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
    float curvature = 2*easting_vehicle/(point_dist*point_dist);

    //given curvature find velocities
    float rad_curvature = 1/curvature;
    if(rad_curvature >= 0.0) {
      float v_left = curr_max_vel/2.0;
      ang_vel = v_left/(rad_curvature + L/2.0);
      lin_vel = ang_vel*rad_curvature;
    }
    else {
      float v_right = curr_max_vel/2.0;
      ang_vel = v_right/(rad_curvature - L/2.0);
      lin_vel = ang_vel*rad_curvature;      
    }
        
    PublishSpeed(lin_vel, ang_vel);
    finished_turning_in_place = false;
    loop_rate.sleep();
  }

  //tell the state controller you are done tracking
  printf("outside of track loop\n");
  PublishSpeed(0.0, 0.0);
}

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::OdomCallback(nav_msgs::Odometry msg) {
  //odom_msg = msg;
  should_start = true;
  odom_x = msg.pose.pose.position.x;
  odom_y = msg.pose.pose.position.y;
  odom_vel = msg.twist.twist.linear.x;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

/***********************************************************************
 *                                                                     *
 *                      PUBLISHER                                      *
 *                                                                     *
 *********************************************************************/
void TrackPoint::PublishSpeed(float lin_vel, float ang_vel) {
  cmd_vel.linear.x = lin_vel;
  cmd_vel.angular.z = ang_vel;
  vel_pub.publish(cmd_vel);
}

/***********************************************************************
 *                                                                     *
 *                      ROS SPIN                                       *
 *                                                                     *
 *********************************************************************/
void TrackPoint::UpdateOdom() {
  ros::spinOnce();
}

/***********************************************************************
 *                                                                     *
 *                      FIND ANGLE ERROR                               *
 *                                                                     *
 *********************************************************************/
float TrackPoint::FindAngleError(float x_des, float y_des) {
  float theta_des, theta_curr, angle_error;
  theta_des = atan2((y_des - odom_y), (x_des - odom_x));
  theta_des = theta_des < 0 ? (2*M_PI + theta_des) : theta_des;
  theta_curr = getYaw(odom_quat);
  theta_curr = theta_curr < 0 ? (2*M_PI + theta_curr) : theta_curr;

  angle_error = theta_des - theta_curr;
  angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
  angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

  return angle_error;
}


/***********************************************************************
 *                                                                     *
 *                      FIND POSITION ERROR                            *
 *                                                                     *
 *********************************************************************/
float TrackPoint::FindPositionError(float x_des, float y_des) {
  float position_error;
  position_error = sqrt(pow((odom_x - x_des),2) + pow((odom_y - y_des),2));

  return position_error;
}


/***********************************************************************
 *                                                                     *
 *               FIND DISTANCE YOU SHOULD LOOK AHEAD                   *
 *                                                                     *
 *********************************************************************/
float TrackPoint::FindLookAheadDistance() {
  float curr_vel = odom_vel;
  float min_vel = 0.5;
  float max_vel = 0.9;
  float lower_limit = min_vel + 0.1*min_vel;
  float upper_limit = max_vel - 0.1*max_vel;

  float min_look_ahead = 2.0;
  float max_look_ahead = 6.0;
  
  if(odom_vel < lower_limit) return min_look_ahead; //2 meters
  else if(odom_vel > upper_limit) return max_look_ahead; //6 meters?
  else {
    return (min_look_ahead + (max_look_ahead - min_look_ahead)*(odom_vel - min_vel)/(max_vel - min_vel));
  }
}

/***********************************************************************
 *                                                                     *
 *               FIND CARS CURRENT MAX VELOCITY                        *
 *                                                                     *
 *********************************************************************/
float TrackPoint::FindCurrMaxVel(float _vel2, float _vel1, float _dist2, float _dist1) {
  float vel1, vel2, dist1, dist2;
  vel2 = _vel2;
  vel1 = _vel1;
  dist2 = _dist2;
  dist1 = _dist1;

  float rate = 0.1; //increase in velocity per meter you move away from corners
  float abs_max_vel = 1.0; //dont go higher than 1 m/s for now
  float vel_dist_1 = vel1 + rate*dist1;
  float vel_dist_2 = vel2 + rate*dist2;

  float final_vel = (vel_dist_1 <= vel_dist_2)*vel_dist_1 + (vel_dist_2 < vel_dist_1)*vel_dist_2;
  if(final_vel > abs_max_vel) return abs_max_vel;
  return final_vel;
}


/***********************************************************************
 *                                                                     *
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double TrackPoint::WrapAngle( double angle ) {
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}






















// /***********************************************************************
//  *                                                                     *
//  *         FIND THE POINT CLOSEST TO YOUR CURRENT POSITION             *
//  *                                                                     *
//  *********************************************************************/
// int TrackPoint::FindClosestPointIndex(double *northing_array, double *easting_array, int _array_size) {
//   int min_index;
//   int array_size = _array_size;
//   float min_distance = 1000.0;
//   float curr_distance;
  
//   for(int i = 0; i < array_size; i++) {
//     curr_distance = FindPositionError(northing_array[i], easting_array[i]);
//     if (curr_distance < min_distance) {
//       min_distance = curr_distance;
//       min_index = i;
//     }
//   }

//   return min_index;
// }



// /***********************************************************************
//  *                                                                     *
//  *                             MAIN TASK                               *
//  *                                                                     *
//  *********************************************************************/
// void TrackPoint::Task() {

//   //wait for odom data before starting this function
//   while(!should_start) {
//     UpdateOdom();
//   }
  
//   float tracking_distance = 1.0; //how far down the path the point we are tracking is
  
//   //import path file
//   std::ifstream inFile, velFile;
//   std::string str_northing, str_easting, str_vel;
//   double des_northing, des_easting, point_dist;
//   double curvature, ang_vel, lin_vel;
//   float angle_error;
//   bool finished_turning_in_place = false;

//   float L, max_vel, sim_time, resolution;

//   L = 0.4; //meters
//   max_vel = 0.5; //m/s
//   sim_time = 3; //seconds
//   resolution = 0.25; //seconds

//   int array_size = 10;
//   int iterator_count = 0;
//   int closest_point_index = 0;
//   double northing_array[array_size];
//   double easting_array[array_size];
//   float vel_array[array_size];
//   float distance_array[array_size];
//   float curr_max_vel = 0.5;
  
//   // nh.getParam("/vehicle_width", L);
//   // nh.getParam("/max_velocity", max_vel);
//   // nh.getParam("/sim_time_collision", sim_time);
//   // nh.getParam("/resolution", resolution);

//   char *pEnd;
  
//   inFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");
//   if (!inFile) {
//     std::cout << "unable to open path file" << std::endl;
//   }

//   velFile.open("/home/robot/catkin_ws/src/testing/gps_files/vel_map.txt");
//   if (!velFile) {
//     std::cout << "unable to open path file" << std::endl;
//   }

//   //Initialize a list of the first ten path points and velocities
//   for(int i = 0; i < array_size; i++) {
//     std::getline(inFile, str_northing);
//     std::getline(inFile, str_easting);
//     std::getline(velFile, str_vel);
//     northing_array[i] = strtof(str_northing.c_str(), &pEnd);
//     easting_array[i] = strtof(str_easting.c_str(), &pEnd);
//     vel_array[i] = strtof(str_vel.c_str(), &pEnd);
//   }
  
//   des_northing = northing_array[0];
//   des_easting = easting_array[0];
  
//   ros::Rate loop_rate(10);
  
//   while((iterator_count != array_size) && ros::ok()) {
//     //update odometry
//     UpdateOdom();

//     //if collision is detected let the local planner take over and wait for it to finish
//     if(collisionObject->Task(sim_time, resolution)) {
//       printf("collision detected\n");
//       GOAL_X = des_northing - odom_x;
//       GOAL_Y = des_easting - odom_y;
//       ORIENTATION = getYaw(odom_quat);

//       COLLISION_DETECTED = true;
//       PublishSpeed(0.0, 0.0);
//       while(COLLISION_DETECTED) {
// 	ros::Duration(1.0).sleep();
//       }
//       UpdateOdom();
//     }

//     //update tracking distance based on current velocity
//     tracking_distance = FindLookAheadDistance();
//     point_dist = FindPositionError(des_northing, des_easting);

//     //This while loop updates the point you are tracking
//     while(point_dist < tracking_distance) {
//       iterator_count += 1;
//       if(iterator_count == array_size) break;
//       des_northing = northing_array[iterator_count];
//       des_easting = easting_array[iterator_count];
//       closest_point_index = FindClosestPointIndex(northing_array, easting_array, array_size);
//       curr_max_vel = vel_array[closest_point_index];
//       point_dist = FindPositionError(des_northing, des_easting);
//     }
    
//     //this while loop updates the vector of points and allowable velocities
//     while(FindPositionError(northing_array[array_size - 1], easting_array[array_size - 1]) < (tracking_distance + tracking_distance/2.0)) {
//       std::getline(inFile, str_northing);
//       if(inFile.eof()) {break;}
//       std::getline(inFile, str_easting);
//       std::getline(velFile, str_vel);

//       for(int i = 0; i < (array_size - 1); i++) {
// 	northing_array[i] = northing_array[i + 1];
// 	easting_array[i] = easting_array[i + 1];
// 	vel_array[i] = vel_array[i + 1];
//       }
      
//       northing_array[array_size - 1] = strtof(str_northing.c_str(), &pEnd);
//       easting_array[array_size - 1] = strtof(str_easting.c_str(), &pEnd);
//       vel_array[array_size - 1] = strtof(str_vel.c_str(), &pEnd);
//       iterator_count -= 1;
//     }
    
//     //Now that everything is updated you are ready to start sending command velocities
//     angle_error = FindAngleError(des_northing, des_easting);

//     //if you are facing the wrong direction turn around
//     if(abs(angle_error) > (70.0*M_PI/180.0)) {
//       while(!finished_turning_in_place && ros::ok()) {

//     	//adjust angular velocity and publish speed
//     	//ang_vel = angle_error*turn_in_place_KP;
// 	if(angle_error > 0) PublishSpeed(0.0, 0.5);
// 	else PublishSpeed(0.0, -0.5);

//     	//update odometry
//     	UpdateOdom();

//     	//update error
//     	angle_error = FindAngleError(des_northing, des_easting);

//     	finished_turning_in_place = abs(angle_error*180/M_PI) < 8.0 ? true : false;
//       }
//       PublishSpeed(0.0, 0.0);
//     }
    
//     //find the curvature given the point - 1/R = 2x/D^2
//     float theta_curr = getYaw(odom_quat);// + imu_drift;
//     float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
//     float curvature = 2*easting_vehicle/(point_dist*point_dist);

//     //given curvature find velocities
//     float rad_curvature = 1/curvature;
//     if(rad_curvature >= 0.0) {
//       float v_left = curr_max_vel/2.0;
//       ang_vel = v_left/(rad_curvature + L/2.0);
//       lin_vel = ang_vel*rad_curvature;
//     }
//     else {
//       float v_right = curr_max_vel/2.0;
//       ang_vel = v_right/(rad_curvature - L/2.0);
//       lin_vel = ang_vel*rad_curvature;      
//     }
        
//     PublishSpeed(lin_vel, ang_vel);
//     finished_turning_in_place = false;
//     loop_rate.sleep();
//   }

//   //tell the state controller you are done tracking
//   printf("outside of track loop\n");
//   PublishSpeed(0.0, 0.0);
// }
