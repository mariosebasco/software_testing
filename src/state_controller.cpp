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
ros::Publisher vel_pub;

void PathCB(const testing::Path_msg &msg);
void PublishSpeed(float lin_vel, float rot_vel);

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "sample_node");
  ros::NodeHandle nh;
  ros::Subscriber path_sub = nh.subscribe("vehicle_path", 1, PathCB);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
  Collision *collisionObject = new Collision();
  DWA *dwaObject = new DWA(collisionObject);
  TrackPoint *trackPointObject = new TrackPoint();
  Video *videoObject = new Video();
  A_star *aStarObject = new A_star();
  Path *pathObject = new Path();
  
  bool will_collide;

  float sim_time, resolution;
  int prev_id = 0;
  sim_time = 3; //seconds
  resolution = 0.25; 

  pathObject->Init();
  trackPointObject->Init();
  aStarObject->Init();
  dwaObject->Init();
  
  ros::Rate loop_rate(10.0);
  //while(ros::ok()) {loop_rate.sleep();}
  while(ros::ok()) {
    while(!received_path && ros::ok()) ros::spinOnce();
    
    switch(StateController::vehicle_state) {
    case IDLE:
      StateController::vehicle_state = TRACKING_GLOBAL;
      ROS_INFO("switching to global state\n");	      
      break;
      
    case TRACKING_GLOBAL:
      collisionObject->UpdateCallbacks();
      if(collisionObject->Task(sim_time, resolution)) {
	ROS_INFO("switching to local state\n");
  	StateController::vehicle_state = TRACKING_LOCAL;
	break;
      }
      if(path_msg.reached_end) {
	StateController::vehicle_state = FINISHED;
	ROS_INFO("switching to finished state\n");
	break;
      }
      if(path_msg.event_point) {
	if(path_msg.event_id != prev_id) {
	  ROS_INFO("switching to event state\n");
	  StateController::vehicle_state = RECORDING_VIDEO;
	  videoObject->Init(path_msg.event_duration, path_msg.event_id);
	  prev_id = path_msg.event_id;
	  break;
	}
      }
      trackPointObject->Trigger(0);
      break;
      
    case TRACKING_LOCAL:
      if(dwaObject->NO_OBSTACLE_FOUND) {
	dwaObject->NO_OBSTACLE_FOUND = false;
      	StateController::vehicle_state = TRACKING_GLOBAL;
	ROS_INFO("switching to global state\n");	
	break;
      }
      if(path_msg.reached_end) {
	StateController::vehicle_state = FINISHED;
	ROS_INFO("switching to finished state\n");		
	break;
      }
      if(path_msg.event_point) {
	if(path_msg.event_id != prev_id) {
	  StateController::vehicle_state = RECORDING_VIDEO;
	  ROS_INFO("switching to event state\n");	  
	  videoObject->Init(path_msg.event_duration, path_msg.event_id);
	  prev_id = path_msg.event_id;
	  break;
	}
      }
      aStarObject->Trigger(0);
      dwaObject->Trigger(0);
      break;
      
    case RECORDING_VIDEO:
      if(videoObject->FINISHED_RECORDING) {
	ROS_INFO("switching to global state\n");		
	StateController::vehicle_state = TRACKING_GLOBAL;
      }
      break;
      
    case FAULT:
      //subscribe to fault topic... wait for the topic to give you the green light to continue?
      break;
      
    case FINISHED:
      PublishSpeed(0.0, 0.0);
      return 0;
      
    default:
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

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

void PublishSpeed(float lin_vel, float rot_vel) {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = lin_vel;
  cmd_vel.angular.z = rot_vel;
  vel_pub.publish(cmd_vel);
}
