/*
 * 
 *
 *
 */


#ifndef STATE_CONTROLLER_H
#define STATE_CONTROLLER_H

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#include "track.h"
#include "collision.h"
#include "video.h"
#include "dwa.h"
#include "A_star.h"
#include "path.h"

#include "testing/Path_msg.h"

enum VehicleState {IDLE, TRACKING_GLOBAL, TRACKING_LOCAL, RECORDING_VIDEO, FAULT, FINISHED};

class StateController {
 public:
  static VehicleState vehicle_state;

 private:
};

#endif
