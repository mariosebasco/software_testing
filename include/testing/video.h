/*
 *
 *
 *
 *
 */

#ifndef VIDEO_H
#define VIDEO_H

#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "AperiodicTask.h"
#include "state_controller.h"

class Video: public AperiodicTask {
public:
  bool FINISHED_RECORDING;
  
  Video();
  int Init(double duration_input, int video);

private:
  double duration;
  double northing, easting;
  int video_number;
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;

  
    
  void Task();  
  void start();
  void end();

  void odomCallback(nav_msgs::Odometry msg);
  
};

#endif
