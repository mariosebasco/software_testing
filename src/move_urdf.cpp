#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom_data;

void callback(nav_msgs::Odometry msg) {
  odom_data = msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_transform");

  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("odom", 1, callback);
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
 
  ros::Time current_time;

  ros::Rate loop_rate(50);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);  
  while(ros::ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
  
    // //odom to base footprint
    // odom_trans.header.stamp = current_time;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_footprint";

    // odom_trans.transform.translation.x = odom_data.pose.pose.position.x;
    // odom_trans.transform.translation.y = odom_data.pose.pose.position.y;
    // odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation = odom_data.pose.pose.orientation;

    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

    //base footprint to chassis
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_footprint";
    odom_trans.child_frame_id = "chassis";

    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //chassis to laser
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "chassis";
    odom_trans.child_frame_id = "hokuyo_link";

    odom_trans.transform.translation.x = 0.3;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.227;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    loop_rate.sleep();
  }
}
