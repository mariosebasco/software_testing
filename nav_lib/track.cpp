/*
 *
 *  Path follower - 
 *  INPUT: desired poses
 *  OUTPUT: command velocity
 *
 */


#include "track.h"


/***********************************************************************
 *                                                                     *
 *                      CONSTRUCTOR                                    *
 *                                                                     *
 *********************************************************************/
TrackPoint::TrackPoint() {
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odom_sub = nh.subscribe("odom", 1, &TrackPoint::odom_callback, this);    
  should_start = false;
  goal_interrupt = false;
  angle_KP = 0.85;
  position_KP = 0.85;
}

/***********************************************************************
 *                                                                     *
 *                      MAIN TRACKING FUNCTION                         *
 *                                                                     *
 *********************************************************************/
void TrackPoint::tracker(geometry_msgs::Pose *desired_poses) {

  bool finished = false;
  bool finished_turning_in_place = false;
  float position_error, angle_error, ang_vel, lin_vel, lin_vel1, lin_vel2;
  float position_error2, angle_error2;

  //calculate error
  angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);
  position_error = findPositionError(desired_poses[0].position.x, desired_poses[0].position.y);

  angle_error2 = findAngleError(desired_poses[1].position.x, desired_poses[1].position.y);
  position_error2 = findPositionError(desired_poses[1].position.x, desired_poses[1].position.y);

  //move accordingly
  //if you are facing opposite direction as target, turn to face it first
  if(abs(angle_error) > (M_PI / 2)) {
    while(!finished_turning_in_place && !goal_interrupt && ros::ok()) {

      //adjust angular velocity and publish speed
      ang_vel = angle_error*angle_KP;
      publishSpeed(0.0, ang_vel);

      //update odometry
      updateOdom();

      //update error
      angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);

      //Check if error is small enough to escape loop
      //std::cout << angle_error*180/M_PI << std::endl;
      finished_turning_in_place = abs(angle_error*180/M_PI) < 5.0 ? true : false;
    }
  }

  //start moving towards target
  while(!finished && !goal_interrupt && ros::ok()) {

    //adjust velocities and publish speed
    ang_vel = 0.75*angle_error*angle_KP + 0.25*angle_error2*angle_KP;
      
    lin_vel1 = position_error*position_KP;
    lin_vel2 = position_error2*position_KP;
      
    lin_vel1 = lin_vel1 > 0.5 ? 0.5 : lin_vel1;
    lin_vel2 = lin_vel2 > 0.5 ? 0.5 : lin_vel2;
      
    lin_vel = 0.75*lin_vel1 + 0.25*lin_vel2;
    //lin_vel = lin_vel - abs(ang_vel)*0.5;
      
    publishSpeed(lin_vel, ang_vel);
      
    //std::cout << lin_vel << std::endl;
    //std::cout << ang_vel << std::endl;

    //update odometry
    updateOdom();
      
    //update error
    angle_error = findAngleError(desired_poses[0].position.x, desired_poses[0].position.y);
    position_error = findPositionError(desired_poses[0].position.x, desired_poses[0].position.y);

    angle_error2 = findAngleError(desired_poses[1].position.x, desired_poses[1].position.y);
    position_error2 = findPositionError(desired_poses[1].position.x, desired_poses[1].position.y);
    //std::cout << angle_error*180.0/M_PI <<std:: endl;
    //std::cout << position_error <<std:: endl;
      
    //check if error is small enough to escape loop
    if(position_error < 0.50) {finished = true;}

  }    
}

/***********************************************************************
 *                                                                     *
 *                    ODOM CALLBACK FUNCTION                           *
 *                                                                     *
 *********************************************************************/
void TrackPoint::odom_callback(nav_msgs::Odometry msg) {
  should_start = true;
  odom_x = msg.pose.pose.position.x;
  odom_y = msg.pose.pose.position.y;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}

/***********************************************************************
 *                                                                     *
 *                      PUBLISHER                                      *
 *                                                                     *
 *********************************************************************/
void TrackPoint::publishSpeed(float lin_vel, float ang_vel) {
  cmd_vel.linear.x = lin_vel;
  cmd_vel.angular.z = ang_vel;
  vel_pub.publish(cmd_vel);
}

/***********************************************************************
 *                                                                     *
 *                      ROS SPIN                                       *
 *                                                                     *
 *********************************************************************/
void TrackPoint::updateOdom() {
  ros::spinOnce();
}

/***********************************************************************
 *                                                                     *
 *                      FIND ANGLE ERROR                               *
 *                                                                     *
 *********************************************************************/
float TrackPoint::findAngleError(float x_des, float y_des) {
  float theta_des, theta_curr, angle_error;
  theta_des = atan2((y_des - odom_y), (x_des - odom_x));
  theta_des = theta_des < 0 ? (2*M_PI + theta_des) : theta_des;
  theta_curr = getYaw(odom_quat);
  theta_curr = theta_curr < 0 ? (2*M_PI + theta_curr) : theta_curr;

  angle_error = theta_des - theta_curr;
  angle_error = angle_error > M_PI ? (angle_error - 2*M_PI) : angle_error;
  angle_error = angle_error < -M_PI ? (angle_error + 2*M_PI) : angle_error;

  return angle_error;
}

/***********************************************************************
 *                                                                     *
 *                      FIND POSITION ERROR                            *
 *                                                                     *
 *********************************************************************/
float TrackPoint::findPositionError(float x_des, float y_des) {
  float position_error;
  position_error = sqrt(pow((odom_x - x_des),2) + pow((odom_y - y_des),2));

  return position_error;
}

  

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
