/*
 * In this file
 * STATE 1: driving through street no obstacle
 * STATE 2: obstacle detected in road, try and go around it
 * STATE 3: navigate in populated area (ex. inside the farm where's there are houses, etc.)
 * 
 * Set up a STATE 0 whose objective it is to get setup and reach STATE 1?
 * setup a service in order to continually update goal poses from a text file
 *
 *
 */


#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#include <iostream>
#include <fstream>

#include <nav_msgs/OccupancyGrid.h>

#include "track.h"
#include "collision.h"
#include "video.h"
#include "transform_gps.h"

enum CollisionStates {CRASH, NO_CRASH};
enum TrackerStates {TRACKING, NOT_TRACKING};
enum VideoStates {RECORDING, NOT_RECORDING};


class StateController {
 public:
  static CollisionStates collision_state;
  static TrackerStates tracker_state;
  static VideoStates video_state;

 private:
};

#endif
