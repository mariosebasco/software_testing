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

#include "sim_track.h"
#include "sim_collision.h"
#include "sim_dwa.h"
#include "sim_A_star.h"
#include "sim_path.h"

#include "testing/Path_msg.h"

enum VehicleState {IDLE, TRACKING_GLOBAL, TRACKING_LOCAL, RECORDING_VIDEO, FAULT, FINISHED};

class StateController {
 public:
  static VehicleState vehicle_state;

 private:
};

#endif
