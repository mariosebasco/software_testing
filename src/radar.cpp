#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "testing/EsrTrack.h"

sensor_msgs::LaserScan laser_scan;
//sensor_msgs::LaserScan temp_scan;
testing::EsrTrack radar_scan;
bool received_laser_scan = false;
//ros::Publisher laser_pub;

void LaserCB(sensor_msgs::LaserScan msg);
void RadarCB(testing::EsrTrack msg);

/************************************************************************************************
 *                                                                                              *
 *                                         MAIN                                                 *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_node");
  ros::NodeHandle nh;
  ros::Subscriber lidar_sub = nh.subscribe("scan", 1, LaserCB);
  ros::Subscriber radar_sub = nh.subscribe("parsed_tx/radartrack", 1, RadarCB);
  //laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan_new", 1000);

  ros::Rate loop_rate(2000);
  
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


/************************************************************************************************
 *                                                                                              *
 *                          LASER CALLBACK FUNCTION                                             *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
void LaserCB(sensor_msgs::LaserScan msg) {
  received_laser_scan = true;
  laser_scan = msg;
  // temp_scan = msg;
  // for (int i = 0; i < temp_scan.ranges.size(); i++) {
  //   temp_scan.ranges[i] = 0.0;
  //   temp_scan.intensities[i] = 0.0;
  // }
}


/************************************************************************************************
 *                                                                                              *
 *                                  RADAR CALLBACK FUNCTION                                     *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
void RadarCB(testing::EsrTrack msg) {
  if(!received_laser_scan) return;

  if (msg.track_range > 0.0 && msg.track_range < 30.0) {
      
    float radar_theta = msg.track_angle*M_PI/180.0;
    int index_low = floor((radar_theta - laser_scan.angle_min - 5.0*M_PI/180.0)/laser_scan.angle_increment);
    int index_hi = ceil((radar_theta - laser_scan.angle_min + 5.0*M_PI/180.0)/laser_scan.angle_increment);	
    int num_its = index_hi - index_low;
    bool mismatch = true;
    for (int i = index_low; i < (index_low + num_its); i++) {
      if(msg.track_range - laser_scan.ranges[i] > -0.2) {
	mismatch = false;
	break;
      }
    }
      
    if(mismatch) {
      // for (int i = index_low; i < (index_low + num_its); i++) {
      // 	temp_scan.ranges[i] = laser_scan.ranges[i];
      // 	temp_scan.intensities[i] = 100.0;
      // }
      // printf("********************************\n");
      // printf("radar range: %f\n", msg.track_range);
      // printf("radar angle: %f\n", msg.track_angle);
      // printf("index_low: %d\n", index_low);
      // printf("index_hi: %d\n", index_hi);
      // printf("********************************\n");
      ROS_WARN("MISMATCH BETWEEN RADAR AND LIDAR DATA\n");
    }
    //laser_pub.publish(temp_scan);
  }
}
