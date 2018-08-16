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
  odom_sub = nh.subscribe("odom", 1, &Video::odomCallback, this);
}

int Video::Init(double duration_input, int video) {
  duration = duration_input;
  video_number = video;
  FINISHED_RECORDING = false;
  
  return AperiodicTask::Init((char *) "VideoTask", 30);
}

void Video::Task() {
  start();

  ros::Duration(duration).sleep();

  end();
  FINISHED_RECORDING = true;
}

void Video::start() {
  int ret;
  char message[100];
  sprintf(message, "roslaunch testing camera_streaming.launch video_number:=%d &", video_number);
  
  ret = system(message);
}

void Video::end() {
  int ret;

  ros::spinOnce();

  ret = system("rosnode kill /camera_recorder");
  ret = system("rosnode kill /camera_node");

  double time_now = ros::Time::now().toSec();

  char message[200];
  sprintf(message, "echo 'video %d: --- northing: %f, easting: %f, time: %f\n' >> /home/robot/catkin_ws/src/testing/videos/video_file.txt", video_number, northing, easting, time_now);
  ret = system(message);
}

void Video::odomCallback(nav_msgs::Odometry msg) {
  northing = msg.pose.pose.position.x;
  easting = msg.pose.pose.position.y;
}
