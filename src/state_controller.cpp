/*
 * 
 * 
 * State Controller - 
 * 
 * 
 * 
 * 
 *
 *
 */


#include "state_controller.h"
#include <string>

VehicleState StateController::vehicle_state = IDLE;
testing::Path_msg path_msg;
bool received_path = false;

void PathCB(const testing::Path_msg &msg);


/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");
  ros::NodeHandle nh;
  ros::Subscriber path_sub = nh.subscribe("vehicle_path", 1, PathCB);
  
  Collision *collisionObject = new Collision();
  DWA *dwaObject = new DWA(collisionObject);
  TrackPoint *trackPointObject = new TrackPoint();
  Video *videoObject = new Video();
  A_star *aStarObject = new A_star();
  Path *pathObject = new Path();
  
  bool will_collide;
  int ret;

  float sim_time, resolution;
  sim_time = 3; //seconds
  resolution = 0.25; //seconds

  pathObject->Init();
  //dwaObject->Init();
  aStarObject->Init();
  
  ros::Rate loop_rate(10);
  while(ros::ok()) {loop_rate.sleep();}
  // while(ros::ok()) {
  //   while(!received_path) {
  //     ros::spinOnce();
  //   }
    
  //   switch(StateController::vehicle_state) {
  //   case IDLE:
  //     StateController::vehicle_state = TRACKING_GLOBAL;
  //     trackPointObject->Init();
  //     break;
      
  //   case TRACKING_GLOBAL:
  //     collisionObject->UpdateCallbacks();
  //     if(collisionObject->Task(sim_time, resolution)) {
  // 	trackPointObject->INTERRUPT = true;
  // 	StateController::vehicle_state = TRACKING_LOCAL;
  //     	dwaObject->Init();
  // 	aStarObject->Init();
  //     }
  //     if(path_msg.event_point) {
  // 	trackPointObject->INTERRUPT = true;
  // 	StateController::vehicle_state = RECORDING_VIDEO;
  // 	videoObject->Init(path_msg.event_duration, path_msg.event_id);
  //     }
  //     if(path_msg.reached_end) StateController::vehicle_state = FINISHED;
  //     ROS_INFO("tracking global path\n");
  //     break;
      
  //   case TRACKING_LOCAL:
  //     if(dwaObject->REACHED_GOAL) {
  // 	aStarObject->INTERRUPT = true;
  // 	dwaObject->INTERRUPT = true;
  //     	StateController::vehicle_state = TRACKING_GLOBAL;
  // 	trackPointObject->Init();
  //     }
  //     if(path_msg.event_point) {
  // 	aStarObject->INTERRUPT = true;
  // 	dwaObject->INTERRUPT = true;
  // 	StateController::vehicle_state = RECORDING_VIDEO;
  // 	videoObject->Init(path_msg.event_duration, path_msg.event_id);
  //     }
  //     if(path_msg.reached_end) StateController::vehicle_state = FINISHED;
  //     ROS_INFO("tracking local path\n");
  //     break;
      
  //   case RECORDING_VIDEO:
  //     ROS_INFO("taking video\n");
  //     break;
      
  //   case FAULT:
  //     ROS_INFO("fault state\n");
  //     break;
      
  //   case FINISHED:
  //     ROS_INFO("finished everything\n");
  //     return 0;
      
  //   default:
  //     break;
  //   }

  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  
  //take a video for 10 seconds
  // videoObject->Init(10, 2);
  // while(StateController::video_state == NOT_RECORDING) {} //wait for it to start recording
  // while(StateController::video_state == RECORDING) {

  //} //wait for it to finish recording
  //printf("finished recording\n");


  ROS_INFO("EXITING");
  delete trackPointObject;
  delete dwaObject;
  delete collisionObject;
  delete videoObject;
  delete aStarObject;
  delete pathObject;
  
  return 0;
}

	   
void PathCB(const testing::Path_msg &msg) {
  path_msg = msg;
  received_path = true;
}
