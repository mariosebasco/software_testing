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
TrackPoint::TrackPoint() : AperiodicTask() {
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  path_sub = nh.subscribe("vehicle_path", 1, &TrackPoint::PathCB, this);    
  odom_sub = nh.subscribe("odom", 1, &TrackPoint::OdomCB, this);    
}

/***********************************************************************
 *                                                                     *
 *                      INITIALIZATION                                 *
 *                                                                     *
 *********************************************************************/
int TrackPoint::Init() {
  received_odom = false;
  received_path = false;

  return AperiodicTask::Init((char *) "trackTask", 40);
}


/***********************************************************************
 *                                                                     *
 *                             MAIN TASK                               *
 *                                                                     *
 *********************************************************************/
void TrackPoint::Task() {

  //wait for odom data before starting this function
  while((!received_odom || !received_path) && ros::ok()) {
    ros::spinOnce();
  }
  
  double des_northing, des_easting, point_dist, odom_x, odom_y;
  float curvature, ang_vel, lin_vel;
  float angle_error, curr_max_vel;

  float L = 0.4;

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    TriggerWait();
    
    //update goal point and max speed
    des_northing = path_msg.des_northing;
    des_easting = path_msg.des_easting;
    curr_max_vel = path_msg.max_vel;
    if(path_msg.reached_end) break;

    //update odom
    odom_x = odom_msg.pose.pose.position.x;
    odom_y = odom_msg.pose.pose.position.y;
    
    //find the curvature given the point - 1/R = 2x/D^2
    point_dist = sqrt(pow(odom_x - des_northing, 2) + pow(odom_y - des_easting, 2));
    float theta_curr = getYaw(odom_quat);// + imu_drift;
    float easting_vehicle = (des_northing - odom_x)*sin(-theta_curr) + (des_easting - odom_y)*cos(-theta_curr);
    float curvature = 2*easting_vehicle/(point_dist*point_dist);
    float rad_curvature;
    
    //given curvature find velocities
    if(curvature == 0.0) {
      ang_vel = 0.0;
      lin_vel = curr_max_vel;
    }
    else if(curvature > 0.0) {
      rad_curvature = 1/curvature;
      float v_left = curr_max_vel;
      ang_vel = v_left/(rad_curvature + L/2.0);
      lin_vel = ang_vel*rad_curvature;
    }
    else {
      rad_curvature = 1/curvature;
      float v_right = curr_max_vel;
      ang_vel = v_right/(rad_curvature - L/2.0);
      lin_vel = ang_vel*rad_curvature;      
    }
        
    PublishSpeed(lin_vel, ang_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //You are done tracking
  PublishSpeed(0.0, 0.0);
}

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::OdomCB(const nav_msgs::Odometry &msg) {
  received_odom = true;
  odom_msg = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}


/***********************************************************************
 *                                                                     *
 *                    PATH CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::PathCB(const testing::Path_msg &msg) {
  received_path = true;
  path_msg = msg;
}

/***********************************************************************
 *                                                                     *
 *                      FIND ANGLE ERROR                               *
 *                                                                     *
 *********************************************************************/
float TrackPoint::FindAngleError(float x_des, float y_des) {
  float theta_des, theta_curr, angle_error;

  double odom_x = odom_msg.pose.pose.position.x;
  double odom_y = odom_msg.pose.pose.position.y;
  
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
  float odom_x = odom_msg.pose.pose.position.x;
  float odom_y = odom_msg.pose.pose.position.y;
  
  position_error = sqrt(pow((odom_x - x_des),2) + pow((odom_y - y_des),2));

  return position_error;
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


/************************************************************************
 *                        PUBLISH VELOCITY                              *
 *                                                                      *
 *                                                                      *
 *                                                                      *          
 ***********************************************************************/
void TrackPoint::PublishSpeed(float trans_vel, float rot_vel) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = trans_vel;
  cmd_vel.angular.z = rot_vel;
  vel_pub.publish(cmd_vel);
}
