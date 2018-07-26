/*
 *
 *  Path follower - 
 *  INPUT: Path
 *  OUTPUT: command velocity
 *
 */


#include "sim_track.h"

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
  int line_count = 1;
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
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/sim_path.txt");
  if (!inFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  velFile.open("/home/robot/catkin_ws/src/testing/gps_files/sim_vel.txt");
  if (!velFile) {
    std::cout << "unable to open vel file" << std::endl;
  }
  
  std::getline(inFile, str_northing);
  std::getline(inFile, str_easting);
  std::getline(velFile, str_vel);
  northings.insert(northings.begin(), strtof(str_northing.c_str(), &pEnd));
  eastings.insert(eastings.begin(), strtof(str_easting.c_str(), &pEnd));
  velocities.push_back(strtof(str_vel.c_str(), &pEnd));

  des_northing = northings[1];
  des_easting = eastings[1];

  angle_error = FindAngleError(des_northing, des_easting);
  printf("angle error: %f\n", angle_error);
  while(!finished_turning_in_place && ros::ok()) {

    //adjust angular velocity and publish speed
    ang_vel = angle_error*turn_in_place_KP;
    if(ang_vel > 0.5) ang_vel = 0.5;
    if(ang_vel < -0.5) ang_vel = -0.5;
    
    PublishSpeed(0.0, ang_vel);

    //update odometry
    UpdateOdom();

    //update error
    angle_error = FindAngleError(des_northing, des_easting);

    finished_turning_in_place = abs(angle_error*180.0/M_PI) < 8.0 ? true : false;
  }
  PublishSpeed(0.0, 0.0);
  finished_turning_in_place = false;
  
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    //update odometry
    UpdateOdom();

    //update odom and costmap for the collision class
    collisionObject->UpdateCallbacks();
    //if collision is detected let the local planner take over and wait for it to finish
    if(collisionObject->Task(sim_time, resolution)) {
      printf("collision detected\n");
      GOAL_X = des_northing;
      GOAL_Y = des_easting;
      PATH_POINT = line_count - 1;
      COLLISION_DETECTED = true;
      //PublishSpeed(0.0, 0.0);

      while(COLLISION_DETECTED) {
    	ros::Duration(1.0).sleep();
      }
      
      UpdateOdom();

      if(line_count < PATH_POINT - 1) {
	std::getline(inFile, str_northing);
	std::getline(inFile, str_easting);
	std::getline(velFile, str_vel);
	line_count += 1;
	while(line_count < PATH_POINT - 1) {
	  std::getline(inFile, str_northing);
	  std::getline(inFile, str_easting);
	  std::getline(velFile, str_vel);
	  line_count += 1;
	}
           
	northings.clear();
	eastings.clear();
	velocities.clear();
	northings.push_back(strtof(str_northing.c_str(), &pEnd));
	eastings.push_back(strtof(str_easting.c_str(), &pEnd));
	velocities.push_back(strtof(str_vel.c_str(), &pEnd));
	std::getline(inFile, str_northing);
	std::getline(inFile, str_easting);
	std::getline(velFile, str_vel);
	northings.insert(northings.begin(), strtof(str_northing.c_str(), &pEnd));
	eastings.insert(eastings.begin(), strtof(str_easting.c_str(), &pEnd));
	velocities.push_back(strtof(str_vel.c_str(), &pEnd));

	vector_size = 2;
      }
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
	  line_count += 1;
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
    angle_error = FindAngleError(des_northing, des_easting);
    point_dist = FindPositionError(des_northing, des_easting);
    
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
	printf("Switched paths!\n");
      }
    }

    if(inFile.eof()) {
      if(dist2 < 1.0) {
	printf("breaking: reached end\n");
	break;
      }
    }
    


    //if you are facing the wrong direction turn around
    if(abs(angle_error) > (70.0*M_PI/180.0)) {
      while(!finished_turning_in_place && ros::ok()) {
    	//adjust angular velocity and publish speed
    	ang_vel = angle_error*turn_in_place_KP;
	if(ang_vel > 0.5) ang_vel = 0.5;
	if(ang_vel < -0.5) ang_vel = -0.5;
	
	PublishSpeed(0.0, ang_vel);

    	//update odometry
    	UpdateOdom();

    	//update error
    	angle_error = FindAngleError(des_northing, des_easting);

    	finished_turning_in_place = abs(angle_error*180.0/M_PI) < 8.0 ? true : false;
      }
      PublishSpeed(0.0, 0.0);
    }
    finished_turning_in_place = false;
    
    //find the curvature given the point - 1/R = 2x/D^2
    float theta_curr = getYaw(odom_quat);// + imu_drift;
    float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
    float curvature = 2*easting_vehicle/(point_dist*point_dist);
        
    //given curvature find velocities
    float rad_curvature = 1/curvature;
    if(rad_curvature >= 0.0) {
      float v_left = curr_max_vel;
      ang_vel = v_left/(rad_curvature + L/2.0);
      lin_vel = ang_vel*rad_curvature;
    }
    else {
      float v_right = curr_max_vel;
      ang_vel = v_right/(rad_curvature - L/2.0);
      lin_vel = ang_vel*rad_curvature;      
    }
        
    PublishSpeed(lin_vel, ang_vel);

    printf("chasing northing: %f\n", northings[vector_size - 2]);
    printf("chasing easting: %f\n", eastings[vector_size - 2]);
    printf("tracking distance: %f\n", tracking_distance);
    printf("curr vel: %f\n", odom_vel);
    printf("Desired northing: %f\n", des_northing - odom_x);
    printf("Desired easting: %f\n", des_easting - odom_y);
    printf("lin vel: %f\n", lin_vel);
    printf("ang vel: %f\n", ang_vel);
    
    loop_rate.sleep();
  }

  //tell the state controller you are done tracking
  printf("outside of track loop\n");
  inFile.close();
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

  //printf("theta:%f\n", theta_curr);
  
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
  float max_vel = 0.75;
  float lower_limit = min_vel + 0.1*min_vel;
  float upper_limit = max_vel - 0.1*max_vel;

  float min_look_ahead = 2.0;
  float max_look_ahead = 3.0;
  
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
 *               FIND DISTANCE FROM CAR TO LINE OF PATH                *
 *                                                                     *
 *********************************************************************/
// float TrackPoint::FindDistToLine(double _x1, double _y1, double _x2, double _y2) {
//   double x1, x2, y1, y2, b, c, a, x_line, y_line, dist;
//   float percent_travelled;

//   x1 = _x1;
//   y1 = _y1;
//   x2 = _x2;
//   y2 = _y2;
  
//   a = (y2 - y1); // line between these two markers
//   b = -(x2 - x1);
//   c = x2*y1 - x1*y2;

//   dist = fabs(a*odom_x + b*odom_y + c)/sqrt(pow(a, 2) + pow(b, 2));
//   // x_line = (b*(b*odom_x - a*odom_y) - a*c)/(pow(a, 2) + pow(b, 2)); // where the car is in this line
//   // y_line = (a*(-b*odom_x + a*odom_y) - b*c)/(pow(a, 2) + pow(b, 2));
//   // dist2 = sqrt(pow((x_line - x2), 2) + pow((y_line - y2), 2)); // dist from car to next lat/lon marker
//   //dist1 = sqrt(pow((x_line - x1), 2) + pow((y_line - y1), 2));

//     return dist;
// }


/***********************************************************************
 *                                                                     *
 *               FIND DISTANCE FROM CAR TO LINE SEGMENT                *
 *                                                                     *
 *********************************************************************/
double TrackPoint::FindDistToSegment(double _x1, double _y1, double _x2, double _y2) {
  double x1, x2, y1, y2, px, py, u, x, y, dx, dy;

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
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double TrackPoint::WrapAngle( double angle ) {
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}


