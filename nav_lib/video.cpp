/*
 *
 *
 *
 *
 */

#include "video.h"
#include <string>
#include <iostream>

Video::Video() : AperiodicTask() {
}

int Video::Init(double duration_input, int video) {
  duration = duration_input;
  video_number = video;
  
  return AperiodicTask::Init((char *) "VideoTask", 30);
}

void Video::Task() {
  StateController::video_state = RECORDING;
  start();

  ros::Duration(duration).sleep();

  end();
  StateController::video_state = NOT_RECORDING;
}

void Video::start() {
  int ret;
  char message[100];
  sprintf(message, "roslaunch testing camera_streaming.launch video_number:=%d &", video_number);
  
  ret = system(message);
}

void Video::end() {
  int ret;

  ret = system("rosnode kill /camera_recorder");
  ret = system("rosnode kill /camera_node");
}
