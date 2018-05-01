/*
 *
 * 
 * 
 * 
 *
 *
 */

#include "transform_gps.h"

/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
TransformGPS::TransformGPS() : AperiodicTask() {}

/***********************************************************************
 *                                                                     *
 *                      INITIALIZATION                                 *
 *                                                                     *
 *********************************************************************/
int TransformGPS::Init() {
  odom_sub = nh.subscribe("odom/gps_raw", 1, &TransformGPS::odomCallback, this);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom/gps", 1);

  received_data = false;
  current_northing = 0.0;
  current_easting = 0.0;
  init_northing = 0.0;
  init_easting = 0.0;
  
  return AperiodicTask::Init((char *) "transform_gps_task", 20);
}

/***********************************************************************
 *                                                                     *
 *                            TASK                                     *
 *                                                                     *
 *********************************************************************/
void TransformGPS::Task() {
  //Start main loop
  ros::Rate loop_rate(10);
  while(ros::ok()) {

    odom_data.pose.pose.position.x -= init_northing;
    odom_data.pose.pose.position.y -= init_easting;
    odom_data.pose.pose.position.z = 0.0;

    odom_pub.publish(odom_data);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/***********************************************************************
 *                                                                     *
 *                      CALLBACK FUNCTION                              *
 *                                                                     *
 *********************************************************************/
void TransformGPS::odomCallback(nav_msgs::Odometry msg) {
  odom_data = msg;
  if(!received_data) {
    init_northing = msg.pose.pose.position.x;
    init_easting = msg.pose.pose.position.y;
    // printf("init northing: %f\n", init_northing);
    // printf("init easting: %f\n", init_easting);
  }
  // current_northing = msg.pose.pose.position.x;
  // current_easting = msg.pose.pose.position.y;
  
  received_data = true;
}
