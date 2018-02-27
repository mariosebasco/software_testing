/*
 *
 * You can get the orientation from the IMU
 * 
 * 
 *
 *
 */

#include <math.h>
#include <stdio.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "testing/encoder_msg.h"

#include "sensor_msgs/Imu.h"

int encoder1_count, encoder2_count;
double THETA = 0.0;

void encoder_callback(testing::encoder_msg msg) {
  encoder1_count = msg.encoder1_count;
  encoder2_count = msg.encoder2_count;
}

void imu_callback(sensor_msgs::Imu imu_data) {
  tf::Quaternion q(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w);
  tf::Matrix3x3 m(q);
  double roll_temp, pitch_temp;
  m.getRPY(roll_temp, pitch_temp, THETA);
  //std::cout << "Yaw: " << THETA << std::endl;    
}

int main(int argc, char *argv[]) {

  //initialize ROS variables
  ros::init(argc, argv, "encoder_to_odom_node");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("encoder_counts", 1, encoder_callback);
  ros::Subscriber imu_sub = nh.subscribe("imu_data", 1, imu_callback);
  ros::Publisher odom_pub;
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

  nav_msgs::Odometry odom_data;
  
  //set variables
  encoder1_count = 0;
  encoder2_count = 0;
  float x, y, vx, vy, vtheta, v, w, v1, v2, w1, w2, wheel_radius, L;
  x = 0.0;
  y = 0.0;
  wheel_radius = 0.085725; //meters
  L = 0.402336; //meters

  /* ************************************************** */
  //tf variables (testing)
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  /* **************************************************  */

  int prev_count1, prev_count2, del_count1, del_count2;
  prev_count1 = encoder1_count;
  prev_count2 = encoder2_count;

  ros::Time current_time, prev_time;
  current_time = ros::Time::now();
  prev_time = current_time;
  
  int TOTAL_TICKS = 8192; //2048;
  float rad_to_tick = 2.0*M_PI/TOTAL_TICKS;
  
  //Start main loop
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    
    ros::spinOnce();
    
    current_time = ros::Time::now();
    
    //encoder count change
    del_count1 = encoder1_count - prev_count1;
    del_count2 = encoder2_count - prev_count2;

    //wheel rotational velocity
    w1 = (del_count1*rad_to_tick)/(current_time - prev_time).toSec();
    w2 = (del_count2*rad_to_tick)/(current_time - prev_time).toSec();

    //wheel forward velocity
    v1 = w1*wheel_radius;
    v2 = w2*wheel_radius;

    //car forward and rotational velocity
    v = (v1 + v2)/2.0;
    w = (v1 - v2)/L;

    //car rotational velocity and position
    vtheta = w;
    //theta += vtheta*(current_time - prev_time).toSec();

    //car x, y velocity
    vx = cos(THETA)*v;
    vy = sin(THETA)*v;

    //car x, y position
    x += vx*(current_time - prev_time).toSec();
    y += vy*(current_time - prev_time).toSec();

    //publish to an odometry message
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(THETA);

    odom_data.header.stamp = current_time;
    odom_data.header.frame_id = "odom";

    //set the position
    odom_data.pose.pose.position.x = x;
    odom_data.pose.pose.position.y = y;
    odom_data.pose.pose.position.z = 0.0;
    odom_data.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_data.child_frame_id = "chassis";
    odom_data.twist.twist.linear.x = vx;
    odom_data.twist.twist.linear.y = vy;
    odom_data.twist.twist.angular.z = vtheta;

    //set the covariances
    odom_data.pose.covariance[0] = 0.001;
    odom_data.pose.covariance[7] = 0.001;
    odom_data.pose.covariance[14] = 0.01;
    odom_data.pose.covariance[21] = 0.01;
    odom_data.pose.covariance[28] = 0.01;
    odom_data.pose.covariance[35] = 0.05;

    odom_data.twist.covariance[0] = 0.001;
    odom_data.twist.covariance[7] = 0.001;
    odom_data.twist.covariance[14] = 0.01;
    odom_data.twist.covariance[21] = 0.01;
    odom_data.twist.covariance[28] = 0.01;
    odom_data.twist.covariance[35] = 0.05;
    
    //publish the message
    odom_pub.publish(odom_data);

    /* **************************************************  */
    // publish tf (testing)
    //base footprint to chassis
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "chassis";

    // odom_trans.transform.translation.x = x;
    // odom_trans.transform.translation.y = y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);    
    /* **************************************************  */

    //update time and counts
    prev_count1 = encoder1_count;
    prev_count2 = encoder2_count;
    prev_time = current_time;

    loop_rate.sleep();
  }

  return 0;
}
