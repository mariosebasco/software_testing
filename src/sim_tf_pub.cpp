#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"


void callback(nav_msgs::Odometry msg) {
  ros::Time current_time;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0); 
  geometry_msgs::TransformStamped odom_trans;
  static tf::TransformBroadcaster odom_broadcaster;
 
  current_time = ros::Time::now();

  //odom to chassis
  //printf("in loop\n");
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "chassis";

  odom_trans.transform.translation.x = msg.pose.pose.position.x;
  odom_trans.transform.translation.y = msg.pose.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation.z = msg.pose.pose.orientation.z;
  odom_trans.transform.rotation.w = msg.pose.pose.orientation.w;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //chassis to laser
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "chassis";
  odom_trans.child_frame_id = "hokuyo_link";

  odom_trans.transform.translation.x = 0.27;
  odom_trans.transform.translation.y = 0.0;
  odom_trans.transform.translation.z = 0.227;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //chassis to camera
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "chassis";
  odom_trans.child_frame_id = "camera_link";

  odom_trans.transform.translation.x = 0.3;
  odom_trans.transform.translation.y = 0.1;
  odom_trans.transform.translation.z = 0.227;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);


  //chassis to IMU
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "chassis";
  odom_trans.child_frame_id = "imu_link";

  odom_trans.transform.translation.x = 0.0;
  odom_trans.transform.translation.y = 0.0;
  odom_trans.transform.translation.z = 0.227;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher_node");

  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, callback);
  
  ros::spin();
    
  return 0;
}
