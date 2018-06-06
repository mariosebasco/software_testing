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
#include "dwa.h"

enum VehicleState {IDLE, TRACKING_GLOBAL, TRACKING_LOCAL, RECORDING_VIDEO, FAULT, FINISHED};

class StateController {
 public:
  static VehicleState vehicle_state;

 private:
};

#endif
