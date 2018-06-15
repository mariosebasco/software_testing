/*
 *
 *  Path follower - 
 *  INPUT: desired poses
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
  
  float tracking_distance = 2.0; //how far down the path the point we are tracking is
  
  //import path file
  std::ifstream inFile, velFile;
  std::string str_northing, str_easting, str_vel;
  double des_northing, des_easting, point_dist;
  double curvature, ang_vel, lin_vel;
  float angle_error;
  bool finished_turning_in_place = false;

  float L, max_vel, sim_time, resolution;

  L = 0.4; //meters
  max_vel = 0.5; //m/s
  sim_time = 3; //seconds
  resolution = 0.25; //seconds

  int array_size = 10;
  int iterator_count = 0;
  double northing_array[array_size];
  double easting_array[array_size];
  float vel_array[array_size];
  float curr_max_vel = 0.5;
  
  // nh.getParam("/vehicle_width", L);
  // nh.getParam("/max_velocity", max_vel);
  // nh.getParam("/sim_time_collision", sim_time);
  // nh.getParam("/resolution", resolution);

  char *pEnd;
  
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");
  if (!inFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  velFile.open("/home/robot/catkin_ws/src/testing/gps_files/vel.txt");
  if (!velFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  //Initialize a list of the first ten path points and velocities
  for(int i = 0; i < array_size; i++) {
    std::getline(inFile, str_northing);
    std::getline(inFile, str_easting);
    std::getline(velFile, str_vel);
    northing_array[i] = strtof(str_northing.c_str(), &pEnd);
    easting_array[i] = strtof(str_easting.c_str(), &pEnd);
    vel_array[i] = strtof(str_vel.c_str(), &pEnd);
  }
  des_northing = northing_array[0];
  des_easting = easting_array[0];
  
  ros::Rate loop_rate(10);
  
  while((iterator_count != array_size) && ros::ok()) {
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

    //update tracking distance based on current velocity
    tracking_distance = FindLookAheadDistance();
    point_dist = FindPositionError(des_northing, des_easting);
      
    while(point_dist < tracking_distance) {
      iterator_count += 1;
      if(iterator_count == array_size) break;
      des_northing = northing_array[iterator_count];
      des_easting = easting_array[iterator_count];
      curr_max_vel = vel_array[iterator_count];
      point_dist = FindPositionError(des_northing, des_easting);
    }
    
    //update the vector of points and allowable velocities
    while(FindPositionError(northing_array[array_size - 1], easting_array[array_size - 1]) < (tracking_distance + tracking_distance/2.0)) {
      std::getline(inFile, str_northing);
      if(inFile.eof()) {break;}
      std::getline(inFile, str_easting);
      std::getline(velFile, str_vel);

      for(int i = 0; i < (array_size - 1); i++) {
	northing_array[i] = northing_array[i + 1];
	easting_array[i] = easting_array[i + 1];
	vel_array[i] = vel_array[i + 1];
      }
      
      northing_array[array_size - 1] = strtof(str_northing.c_str(), &pEnd);
      easting_array[array_size - 1] = strtof(str_easting.c_str(), &pEnd);
      vel_array[array_size - 1] = strtof(str_vel.c_str(), &pEnd);
      iterator_count -= 1;
    }
    

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

  // printf("theta des: %f\n", theta_des);
  // printf("theta current: %f\n", theta_curr);
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

  if(curr_vel <= 0.5) return 2.0;
  else if(curr_vel <=1.0) return 4.0;
  else return 5.0;
}

/***********************************************************************
 *                                                                     *
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double TrackPoint::WrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}


