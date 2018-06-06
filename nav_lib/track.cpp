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
  odom_sub = nh.subscribe("odom", 1, &TrackPoint::odom_callback, this);    
  should_start = false;
  goal_interrupt = false;

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
    updateOdom();
  }
  
  float tracking_distance = 2.0; //how far down the path the point we are tracking is
  
  //import path file
  std::ifstream inFile;
  std::string northing_1, easting_1;
  double des_northing, des_easting, point_dist;
  double curvature, ang_vel, lin_vel;
  float angle_error;
  bool finished_turning_in_place = false;

  float L, max_vel, sim_time, resolution;
  
  nh.getParam("/vehicle_width", L);
  nh.getParam("/max_velocity", max_vel);
  nh.getParam("/sim_time_collision", sim_time);
  nh.getParam("/resolution", resolution);
    
  char *pEnd;
  bool face_path = false;
  
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");
  if (!inFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  std::getline(inFile, northing_1);
  std::getline(inFile, easting_1);
  des_northing = strtof(northing_1.c_str(), &pEnd);
  des_easting = strtof(easting_1.c_str(), &pEnd);
  
  ros::Rate loop_rate(10);
  
  while(!inFile.eof() && ros::ok()) {
    //update odometry
    updateOdom();

    //if collision is detected let the local planner take over and wait for it to finish
    if(collisionObject->Task(sim_time, resolution)) {
      printf("collision detected\n");
      GOAL_X = des_northing - odom_x;
      GOAL_Y = des_easting - odom_y;
      COLLISION_DETECTED = true;
      publishSpeed(0.0, 0.0);
      while(COLLISION_DETECTED) {
	ros::Duration(1.0).sleep();
      }
      updateOdom();
      face_path = true;
    }
    
    //find the coods of the point in the path 'x' meters away from the vehicle
    point_dist = findPositionError(des_northing, des_easting);
  
    while(point_dist < tracking_distance) {
      std::getline(inFile, northing_1);
      if(inFile.eof()) {break;}
      std::getline(inFile, easting_1);
      des_northing = strtof(northing_1.c_str(), &pEnd);
      des_easting = strtof(easting_1.c_str(), &pEnd);
      point_dist = findPositionError(des_northing, des_easting);
    }

    angle_error = findAngleError(des_northing, des_easting);

    //if you are facing the wrong direction turn around
    if(abs(angle_error) > (M_PI / 2) || face_path) {
      while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {

    	//adjust angular velocity and publish speed
    	ang_vel = angle_error*turn_in_place_KP;
    	publishSpeed(0.0, ang_vel);

    	//update odometry
    	updateOdom();

    	//update error
    	angle_error = findAngleError(des_northing, des_easting);

    	finished_turning_in_place = abs(angle_error*180/M_PI) < 8.0 ? true : false;
      }
    }
    
    //find the curvature given the point - 1/R = 2x/D^2
    float theta_curr = getYaw(odom_quat);// + imu_drift;
    float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
    float curvature = 2*easting_vehicle/(point_dist*point_dist);

    //given curvature find velocities
    float rad_curvature = 1/curvature;
    if(rad_curvature >= 0.0) {
      float v_left = max_vel/2.0;
      ang_vel = v_left/(rad_curvature + L/2.0);
      lin_vel = ang_vel*rad_curvature;
    }
    else {
      float v_right = max_vel/2.0;
      ang_vel = v_right/(rad_curvature - L/2.0);
      lin_vel = ang_vel*rad_curvature;      
    }
        
    publishSpeed(lin_vel, ang_vel);
    
    finished_turning_in_place = false;
    face_path = false;
    
    loop_rate.sleep();
  }

  //tell the state controller you are done tracking
  printf("outside of track loop\n");
  publishSpeed(0.0, 0.0);
  //StateController::vehicle_state = FINISHED;
  
}



/***********************************************************************
 *                                                                     *
 *             TURN IN PLACE IF YOU KNOW DESIRED ANGLE                 *
 *                                                                     *
 *********************************************************************/
// void TrackPoint::turnInPlace(double theta_des) {
//   double theta_curr, angle_error, ang_vel;
//   bool finished_turning_in_place = false;

//   while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {
//     theta_curr = getYaw(odom_quat);
//     theta_curr = theta_curr < 0 ? (2*M_PI + theta_curr) : theta_curr;

//     angle_error = theta_des - theta_curr;
//     angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
//     angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

//     //adjust angular velocity and publish speed
//     ang_vel = angle_error*turn_in_place_KP;
//     publishSpeed(0.0, ang_vel);

//     //update odometry
//     updateOdom();

//     finished_turning_in_place = abs(angle_error*180/M_PI) < 5.0 ? true : false;
//   }
// }

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::odom_callback(nav_msgs::Odometry msg) {
  //odom_msg = msg;
  should_start = true;
  odom_x = msg.pose.pose.position.x;
  odom_y = msg.pose.pose.position.y;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

/***********************************************************************
 *                                                                     *
 *                      PUBLISHER                                      *
 *                                                                     *
 *********************************************************************/
void TrackPoint::publishSpeed(float lin_vel, float ang_vel) {
  cmd_vel.linear.x = lin_vel;
  cmd_vel.angular.z = ang_vel;
  vel_pub.publish(cmd_vel);
}

/***********************************************************************
 *                                                                     *
 *                      ROS SPIN                                       *
 *                                                                     *
 *********************************************************************/
void TrackPoint::updateOdom() {
  ros::spinOnce();
}

/***********************************************************************
 *                                                                     *
 *                      FIND ANGLE ERROR                               *
 *                                                                     *
 *********************************************************************/
float TrackPoint::findAngleError(float x_des, float y_des) {
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
float TrackPoint::findPositionError(float x_des, float y_des) {
  float position_error;
  position_error = sqrt(pow((odom_x - x_des),2) + pow((odom_y - y_des),2));

  return position_error;
}


/***********************************************************************
 *                                                                     *
 *                             MISC                                    *
 *                                                                     *
 *********************************************************************/
inline double TrackPoint::wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}


