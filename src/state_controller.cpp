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


#include "state_controller.h"

TrackerStates StateController::tracker_state = NOT_TRACKING;
CollisionStates StateController::collision_state = NO_CRASH;
VideoStates StateController::video_state = NOT_RECORDING;


/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");

  TrackPoint *trackPointObject = new TrackPoint();
  Collision *collisionObject = new Collision();
  Video *videoObject = new Video();
  //TransformGPS *gpsObject = new TransformGPS();
  
  bool will_collide;
  int ret;
  
  double rate = 20.0;
  int priority = 10;
  double move_time = 2.0; //seconds
  double resolution = 0.5; //seconds

  //initialize GPS converter and wait for first reading;
  // ret = gpsObject->Init();
  // while(!gpsObject->received_data) {}
    
  ros::Rate loop_rate(10);

  ret = collisionObject->Init((char *) "collisionTask", rate, priority, move_time, resolution);

  while(ros::ok()) {
    ret = trackPointObject->Init();//gpsObject->init_northing, gpsObject->init_easting);
      
    //wait for the vehicle to start moving
    while(ros::ok() && StateController::tracker_state == NOT_TRACKING) {/**/}
    while(ros::ok() && StateController::tracker_state == TRACKING) {
      //printf("in tracking loop\n");
      if(StateController::collision_state == CRASH) {
	printf("crashed\n");
	trackPointObject->goal_interrupt = true;
	trackPointObject->publishSpeed(0.0, 0.0);
      }
    }
    std::cout << "PATH COMPLETE" <<std::endl;
    break;
    //take a video for 10 seconds
    // videoObject->Init(10, 1);
    // while(StateController::video_state == NOT_RECORDING) {} //wait for it to start recording
    // while(StateController::video_state == RECORDING) {
    // 	trackPointObject->publishSpeed(0.0, 0.2);
    // } //wait for it to finish recording
    // break;

    loop_rate.sleep();
  }

  std::cout << "EXITING" << std::endl;
  delete trackPointObject;
  delete collisionObject;
  delete videoObject;
  
  return 0;
}

	   
