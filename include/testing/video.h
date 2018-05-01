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

#include "AperiodicTask.h"
#include "state_controller.h"

class Video: public AperiodicTask {
public:
  Video();

  int Init(double duration_input, int video);

private:
  double duration;
  int video_number;
  ros::NodeHandle nh;

  void Task();  
  void start();
  void end();
  
};

#endif
