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
TrackPoint::TrackPoint() : AperiodicTask() {
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odom_sub = nh.subscribe("odom", 1, &TrackPoint::odom_callback, this);    
  should_start = false;
  goal_interrupt = false;

  position_KP = 1.0;
  angle_KP = 1.0;
  //turn_in_place_KP = 0.85;

  // nh.getParam("/calibration_value", imu_drift);
  // printf("calibration: %f\n", imu_drift);
}

/***********************************************************************
 *                                                                     *
 *                      INITIALIZATION                                 *
 *                                                                     *
 *********************************************************************/
int TrackPoint::Init() {//double northing_input, double easting_input) {

  return AperiodicTask::Init((char *) "trackTask", 11);
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
  
  float tracking_distance = 4.0; //how far down the path the point we are tracking is
  
  //import path file
  std::ifstream inFile;
  std::string northing_1, easting_1;
  double des_northing, des_easting, point_dist;
  double curvature, ang_vel, lin_vel;
  float angle_error;
  bool finished_turning_in_place = false;
    
  char *pEnd;
  
  inFile.open("/home/robot/catkin_ws/src/testing/gps_files/path.txt");
  if (!inFile) {
    std::cout << "unable to open path file" << std::endl;
  }

  //tell the state controller you are now tracking
  StateController::tracker_state = TRACKING;

  std::getline(inFile, northing_1);
  std::getline(inFile, easting_1);
  des_northing = strtof(northing_1.c_str(), &pEnd);
  des_easting = strtof(easting_1.c_str(), &pEnd);
  
  ros::Rate loop_rate(5);
  
  while(!inFile.eof() && ros::ok()) {
    //update odometry
    updateOdom();
    
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

    //angle_error = findAngleError(des_northing, des_easting);

    //if you are facing the wrong direction turn around
    // if(abs(angle_error) > (M_PI / 2)) {
    //   while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {

    // 	//adjust angular velocity and publish speed
    // 	ang_vel = angle_error*turn_in_place_KP;
    // 	publishSpeed(0.0, ang_vel);

    // 	//update odometry
    // 	updateOdom();

    // 	//update error
    // 	angle_error = findAngleError(des_northing, des_easting);

    // 	finished_turning_in_place = abs(angle_error*180/M_PI) < 8.0 ? true : false;
    //   }
    // }
    
    //find the curvature given the point - 1/R = 2x/D^2
    // float theta_curr = getYaw(odom_quat) + imu_drift;
    // float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
    // float curvature = 2*easting_vehicle/(point_dist*point_dist);

    // printf("des northing: %f\n", des_northing);
    // printf("des easting: %f\n", des_easting);
    // ang_vel = curvature*angle_KP;
    // lin_vel = 0.5;

    //printf("ang_vel: %f\n", ang_vel);
    
    // publishSpeed(lin_vel, ang_vel);

    DWA(des_northing, des_easting);
    
    //finished_turning_in_place = false;
    loop_rate.sleep();
  }

  //tell the state controller you are done tracking
  printf("outside of while loop\n");
  publishSpeed(0.0, 0.0);
  StateController::tracker_state = NOT_TRACKING;
  
}

/***********************************************************************
 *                                                                     *
 *                     DYNAMIC WINDOW APPROACH                         *
 *                                                                     *
 *********************************************************************/
void TrackPoint::DWA(double des_northing, double des_easting) {
  float cost, temp_cost;
  float lin_vel, ang_vel, neg_ang_vel, optimal_lin_vel, optimal_ang_vel;
  float alpha, beta;
  float max_lin_vel = 0.5; // m/s
  float max_ang_vel = 2.486; // rad/sec
  int beta_max = 4;
  int alpha_max = 10;

  cost = 1000.0;
  
  for(int i = 1; i <= beta_max; i++) {
    beta = i/float(beta_max);
    for(int j = 0; j <= alpha_max; j++) {
      alpha = j/float(alpha_max);
      
      lin_vel = beta*alpha*max_lin_vel;
      ang_vel = beta*(1.0 - alpha)*max_ang_vel;
      neg_ang_vel = -ang_vel;
      
      temp_cost = propagateState(des_northing, des_easting, lin_vel, ang_vel, 1.0);
      if(temp_cost < cost) {
  	cost = temp_cost;
  	optimal_lin_vel = lin_vel;
  	optimal_ang_vel = ang_vel;
      }
      temp_cost = propagateState(des_northing, des_easting, lin_vel, neg_ang_vel, 1.0);
      if(temp_cost < cost) {
  	cost = temp_cost;
  	optimal_lin_vel = lin_vel;
  	optimal_ang_vel = neg_ang_vel;
      }
    }
  }

  // printf("des_northing: %f\n", des_northing);
  // printf("des_easting: %f\n", des_easting);

  printf("lin_vel: %f\n", optimal_lin_vel);
  printf("ang_vel: %f\n", optimal_ang_vel);
  publishSpeed(position_KP*optimal_lin_vel, angle_KP*optimal_ang_vel);

  // printf("position cost: %f\n", findPositionCost(des_northing, des_easting, odom_x, odom_y));
  // printf("angle cost: %f\n", findAngleCost(des_northing, des_easting, odom_x, odom_y, 0.0));

}


/***********************************************************************
 *                                                                     *
 *                      PROPAGATE THE STATE                            *
 *                                                                     *
 ***********************************************************************/
float TrackPoint::propagateState(double des_northing, double des_easting, float lin_vel, float ang_vel, float move_time) {
  double curr_theta;
  double vel_x, vel_y, vel_theta;
  double next_x, next_y, next_theta;

  double del_t = 0.25;

  float position_cost, angle_cost, cost;
    
  //get data
  curr_theta = getYaw(odom_quat);
  curr_theta = curr_theta < 0 ? (2*M_PI + curr_theta) : curr_theta;

  vel_x = lin_vel*cos(curr_theta);
  vel_y = lin_vel*sin(curr_theta);
  vel_theta = ang_vel;

  //find the state
  next_x = odom_x + vel_x*del_t;
  next_y = odom_y + vel_y*del_t;
  next_theta = curr_theta + vel_theta*del_t;
  next_theta = wrapAngle(next_theta);
  next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
  vel_x = lin_vel*cos(next_theta);
  vel_y = lin_vel*sin(next_theta);

  int iterations = int(move_time / del_t); 
  
  for(int i = 1; i < iterations; i++) {
    next_x = next_x + vel_x*del_t;
    next_y = next_y + vel_y*del_t;
    next_theta = next_theta + vel_theta*del_t;
    next_theta = wrapAngle(next_theta);
    next_theta = next_theta < 0 ? (2*M_PI + next_theta) : next_theta;
    vel_x = lin_vel*cos(next_theta);
    vel_y = lin_vel*sin(next_theta);
  }


  //the cost of that position is a function of distance to the point and facing the point
  position_cost = findPositionCost(des_northing, des_easting, next_x, next_y);
  angle_cost = findAngleCost(des_northing, des_easting, next_x, next_y, next_theta);
  
  // printf("position cost: %f\n", position_cost);
  // printf("angle cost: %f\n", angle_cost);
  
  cost = position_cost + angle_cost;

  // printf("lin vel: %f, ang vel: %f, cost: %f\n", lin_vel, ang_vel, cost);

  
  return cost;
}



/***********************************************************************
 *                                                                     *
 *             TURN IN PLACE IF YOU KNOW DESIRED ANGLE                 *
 *                                                                     *
 *********************************************************************/
void TrackPoint::turnInPlace(double theta_des) {
  double theta_curr, angle_error, ang_vel;
  bool finished_turning_in_place = false;

  while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {
    theta_curr = getYaw(odom_quat);
    theta_curr = theta_curr < 0 ? (2*M_PI + theta_curr) : theta_curr;

    angle_error = theta_des - theta_curr;
    angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
    angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

    //adjust angular velocity and publish speed
    ang_vel = angle_error*turn_in_place_KP;
    publishSpeed(0.0, ang_vel);

    //update odometry
    updateOdom();

    finished_turning_in_place = abs(angle_error*180/M_PI) < 5.0 ? true : false;
  }


}

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::odom_callback(nav_msgs::Odometry msg) {
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
 *                      FIND ANGLE COST                               *
 *                                                                     *
 *********************************************************************/
float TrackPoint::findAngleCost(double des_northing, double des_easting, double curr_x, double curr_y, double curr_theta) {
  float theta_des, angle_cost, theta_curr;
  theta_des = atan2((des_easting - curr_y), (des_northing - curr_x));
  theta_des = theta_des < 0 ? (2*M_PI + theta_des) : theta_des;
  // theta_curr = getYaw(odom_quat);
  // theta_curr = theta_curr < 0 ? (2*M_PI + theta_curr) : theta_curr;

  // printf("theta des: %f\n", theta_des);
  // printf("theta current: %f\n", theta_curr);
  angle_cost = theta_des - curr_theta;
  angle_cost = angle_cost > M_PI ? (angle_cost - 2*M_PI) : angle_cost;
  angle_cost = angle_cost < -M_PI ? (angle_cost + 2*M_PI) : angle_cost;

  return fabs(angle_cost);
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
 *                      FIND POSITION COST                            *
 *                                                                     *
 *********************************************************************/
float TrackPoint::findPositionCost(double des_northing, double des_easting, double curr_x, double curr_y) {
  float position_cost;
  position_cost = sqrt(pow((des_northing - curr_x),2) + pow((des_easting - curr_y),2));

  return position_cost;
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












/***********************************************************************
 *                                                                     *
 *                      MAIN TRACKING FUNCTION                         *
 *                                                                     *
 *********************************************************************/
// void TrackPoint::Task() {
  
//   bool finished = false;
//   bool finished_turning_in_place = false;
//   float position_error, angle_error, ang_vel, lin_vel, lin_vel1, lin_vel2;
//   float position_error2, angle_error2;

//   //calculate error
//   angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);
//   position_error = findPositionError(desired_poses[0].position.x, desired_poses[0].position.y);

//   angle_error2 = findAngleError(desired_poses[1].position.x, desired_poses[1].position.y);
//   position_error2 = findPositionError(desired_poses[1].position.x, desired_poses[1].position.y);
  
//   StateController::tracker_state = TRACKING;
  
//   //move accordingly
//   //if you are facing opposite direction as target, turn to face it first
//   if(abs(angle_error) > (M_PI / 2)) {
//     while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {

//       //adjust angular velocity and publish speed
//       ang_vel = angle_error*angle_KP;
//       publishSpeed(0.0, ang_vel);

//       //update odometry
//       updateOdom();

//       //update error
//       angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);
//       //printf("%f\n", angle_error);

//       finished_turning_in_place = abs(angle_error*180/M_PI) < 5.0 ? true : false;
//     }
//   }

//   //start moving towards target
//   while(!finished && !goal_interrupt && ros::ok()) {

//     //adjust velocities and publish speed
//     ang_vel = 0.75*angle_error*angle_KP + 0.25*angle_error2*angle_KP;
      
//     lin_vel1 = position_error*position_;
//     lin_vel2 = position_error2*position_KP;
      
//     lin_vel1 = lin_vel1 > 0.5 ? 0.5 : lin_vel1;
//     lin_vel2 = lin_vel2 > 0.5 ? 0.5 : lin_vel2;
      
//     lin_vel = 0.75*lin_vel1 + 0.25*lin_vel2;
//     //lin_vel = lin_vel - abs(ang_vel)*0.5;
    
//     publishSpeed(lin_vel, ang_vel);
      
//     //update odometry
//     updateOdom();
      
//     //update error
//     angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);
//     position_error = findPositionError(desired_poses[0].position.x, desired_poses[0].position.y);

//     angle_error2 = findAngleError(desired_poses[1].position.x, desired_poses[1].position.y);
//     position_error2 = findPositionError(desired_poses[1].position.x, desired_poses[1].position.y);

//     // printf("angle error: %f\n", angle_error);
//     // printf("tracking x: %f\n", desired_poses[0].position.x);
//     // printf("tracking y: %f\n", desired_poses[0].position.y);
    
//     //check if error is small enough to escape loop
//     if(position_error <= 1.0) {finished = true;}
//   }
//   StateController::tracker_state = NOT_TRACKING;
  
// }
