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

VehicleState StateController::vehicle_state = IDLE;

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");

  Collision *collisionObject = new Collision();
  TrackPoint *trackPointObject = new TrackPoint(collisionObject);
  DWA *dwaObject = new DWA(collisionObject);
  Video *videoObject = new Video();
  
  bool will_collide;
  int ret;

  // dwaObject->GOAL_X = 1.0;
  // dwaObject->GOAL_Y = 0.0;
  //dwaObject->Init();
  
  ros::Rate loop_rate(10);
  //while(ros::ok()) {loop_rate.sleep();}
  while(ros::ok()) {

    switch(StateController::vehicle_state) {
    case IDLE:
      StateController::vehicle_state = TRACKING_GLOBAL;
      ret = trackPointObject->Init();
      break;
      
    case TRACKING_GLOBAL:
      if(trackPointObject->COLLISION_DETECTED) {
  	StateController::vehicle_state = TRACKING_LOCAL;
  	dwaObject->GOAL_X = trackPointObject->GOAL_X;
  	dwaObject->GOAL_Y = trackPointObject->GOAL_Y;
  	dwaObject->ORIENTATION = trackPointObject->ORIENTATION;
  	dwaObject->Init();
      }
      printf("tracking global path\n");
      break;
      
    case TRACKING_LOCAL:
      if(dwaObject->REACHED_GOAL) {
  	StateController::vehicle_state = TRACKING_GLOBAL;
  	trackPointObject->COLLISION_DETECTED = false;
      }
      printf("tracking global path\n");
      break;
      
    case RECORDING_VIDEO:
      printf("taking video\n");
      break;
      
    case FAULT:
      printf("fault state\n");
      break;
      
    case FINISHED:
      printf("finished everything\n");
      break;
      
    default:
      break;
    }
      
    loop_rate.sleep();
  }
  
  //take a video for 10 seconds
  // videoObject->Init(10, 2);
  // while(StateController::video_state == NOT_RECORDING) {} //wait for it to start recording
  // while(StateController::video_state == RECORDING) {
    //trackPointObject->publishSpeed(0.0, 0.2);
  //} //wait for it to finish recording
  //printf("finished recording\n");


  std::cout << "EXITING" << std::endl;
  delete trackPointObject;
  delete dwaObject;
  delete collisionObject;
  delete videoObject;
  
  return 0;
}

	   
