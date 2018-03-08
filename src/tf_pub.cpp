#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"

nav_msgs::Odometry odom_data;
int should_start;

void callback(nav_msgs::Odometry msg) {
  should_start = 1;
  odom_data = msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_publisher_node");

  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("odom", 1, callback);
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;

  should_start = 0;

  //******************************* TESTING ***************************
  // ros::Publisher map_pub;
  // ros::Publisher laser_pub;

  // map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
  // laser_pub = nh.advertise<sensor_msgs::LaserScan>("ugv/laser/scan", 1);

  // nav_msgs::OccupancyGrid fake_map;
  // sensor_msgs::LaserScan fake_laser;
  
  //*******************************************************************
  
 
  ros::Time current_time;

  ros::Rate loop_rate(50);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
  //geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(M_PI);
  
  while(ros::ok()){

    ros::spinOnce();
    
    if(should_start) {
      current_time = ros::Time::now();
  
      //odom to base footprint
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "chassis";

      odom_trans.transform.translation.x = odom_data.pose.pose.position.x;
      odom_trans.transform.translation.y = odom_data.pose.pose.position.y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_data.pose.pose.orientation;

      // odom_trans.transform.translation.x = 0.0;
      // odom_trans.transform.translation.y = 0.0;
      // odom_trans.transform.translation.z = 0.0;
      // odom_trans.transform.rotation = odom_quat;
    
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

      //******************************* TESTING ***************************
      // fake_map.header.stamp = current_time;
      // fake_map.header.frame_id = "map";
      // fake_map.info.map_load_time = current_time;
      // fake_map.info.resolution = 100.0;
      // fake_map.info.width = 10.0;
      // fake_map.info.height = 10.0;
      // fake_map.info.origin.orientation.w = 1.0;

      //map_pub.publish(fake_map);
    
      //******************************* TESTING ***************************

    }
    
    loop_rate.sleep();
  }
  return 0;
}
