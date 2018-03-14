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

#include <iostream>

#include "track.h"
#include "nav_msgs/OccupancyGrid.h"

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
// int main(int argc, char **argv) {
//   ros::init(argc, argv, "sample_node");

  
//   TrackPoint *trackPointObject = new TrackPoint();

//   // float x_floats[] = {0.707, 1.0, 0.707, 0.0, -0.707, -1.0, -0.707, 0.0, 0.0}; //CIRCLE TESTING
//   // float y_floats[] = {0.2928, 1.0, 1.707, 2.0, 1.707, 1.0, 0.2928, 0.0, 0.0};

//   float x_floats[] = {1.0, 2.0, 3.0, 4.0, 4.0}; //STRAIGHT LINE TESTING
//   float y_floats[] = {0.0, 0.0, 0.0, 0.0, 0.0};

//   geometry_msgs::Pose desired_poses[2];
  
//   ros::Rate loop_rate(10);
//   bool should_continue = true;
  
//   while(ros::ok()) {
//     trackPointObject->updateOdom();

//     if(trackPointObject->should_start && should_continue) {
//       for(int i = 0; i < sizeof(x_floats)/4; i++) {
// 	desired_poses[0].position.x = x_floats[i];
// 	desired_poses[0].position.y = y_floats[i];
// 	desired_poses[0].orientation.w = 1.0;
	
// 	desired_poses[1].position.x = x_floats[i+1];
// 	desired_poses[1].position.y = y_floats[i+1];
// 	desired_poses[1].orientation.w = 1.0;
	
// 	trackPointObject->tracker(desired_poses);
// 	std::cout << "reached position at desired tolerance" << std::endl;
//       }
//       //should_continue = false;
//     }

//     loop_rate.sleep();
//   }
  
//   std::cout << "EXITING" << std::endl;
//   delete trackPointObject;
  
//   return 0;
// }


void cb(nav_msgs::OccupancyGrid msg) {
  std::cout << msg.header.frame_id <<std::endl;
  std::cout << msg.info.origin.position.x <<std::endl;
  std::cout << msg.info.origin.position.y <<std::endl;
  std::cout << msg.info.resolution <<std::endl;
  std::cout << msg.info.width <<std::endl;
  std::cout << msg.info.height <<std::endl;

  int myInt = 0;
  int count = 0;
  while(myInt == 0) {
    myInt = msg.data[count];
    count ++;
  }
  std::cout << myInt <<std::endl;
  std::cout << count <<std::endl;
  printf("%d\n", msg.data[19949]);
  //std::cout << msg.data[7463] <<std::endl;
  // std::cout << "data" <<std::endl;
  // for(int i = 7464; i < 7500; i++) {
  //   std::cout << msg.data[i] <<std::endl;
  // }
  
}

//testing
int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");
  ros::NodeHandle nh;
  ros::Subscriber grid_sub;
  grid_sub = nh.subscribe("move_base/local_costmap/costmap", 1, cb);
  //grid_sub = nh.subscribe("map", 1, cb);

  ros::Rate loop_rate(0.5);

  while(ros::ok()) {
    ros::spinOnce();
    
    loop_rate.sleep();
  }
  
  return 0;
}
