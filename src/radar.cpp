#include <iostream>
#include <ros/ros.h>
#include "testing/EsrTrack.h"
#include <stdio.h>
#include <vector>
#include <map>
#include <iterator>
#include <math.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
struct TestPath{ //testpath struct
  float x_coord;
  float y_coord;
};
//printpoints variables
TestPath test_path[4];
int rtp = 0;
int marker_c = 0;
float dur = 0.1;
float dist1 = 0.0;

struct RadarPoints {
  float id;
  float x_value;
  float y_value;
};
bool ref_array[64];
RadarPoints radar_points[64];
bool list_is_full = false;

//------------Plot Test Path Points-------------
void printpoints(int var,int rtp, int marker_c, float dur){
  visualization_msgs::Marker path;
  path.header.frame_id = "/map";
  path.header.stamp = ros::Time::now();
  path.ns = "path_viz";
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  path.type = shape;
  path.action = visualization_msgs::Marker::ADD;
  path.id = var;
  //Pose
  if (rtp == 1){//test points
    path.pose.position.x = test_path[var].x_coord;
    path.pose.position.y = test_path[var].y_coord;
    path.pose.position.z = 0;
    path.lifetime = ros::Duration(5);
  }
  else if (rtp == 2){ //radar points
    path.pose.position.x = radar_points[var].x_value;
    path.pose.position.y = radar_points[var].y_value;
    path.pose.position.z = 0;
    path.lifetime = ros::Duration(0.1);
  }
  path.pose.orientation.x = 0.0;
  path.pose.orientation.y = 0.0;
  path.pose.orientation.z = 0.0;
  path.pose.orientation.w = 1.0;
  //scale
  path.scale.x = 1.0;
  path.scale.y = 1.0;
  path.scale.z = 1.0;
  
  //color
  if(marker_c == 1){ //Green
    path.color.r = 0.0f;
    path.color.g = 1.0f;
    path.color.b = 0.0f;
    path.color.a = 1.0;
  }
  else if (marker_c == 2){ //Red
    path.color.r = 1.0f;
    path.color.g = 0.0f;
    path.color.b = 0.0f;
    path.color.a = 1.0;
  }
  else if (marker_c == 3){ //Turquoise
    path.color.r = 0.0f;
    path.color.g = 1.24f;
    path.color.b = 1.75f;
    path.color.a = 1.0;
    std::cout << "printing" << std::endl;
  }
  marker_pub.publish(path);
}
void RadarCallback(testing::EsrTrack msg){
  if( msg.track_ID == 63) list_is_full = true;
  if (msg.track_status > 0 && !list_is_full){
    radar_points[msg.track_ID].id = msg.track_ID;
    radar_points[msg.track_ID].x_value =  msg.track_range*(cos (M_PI*msg.track_angle/180.0));
    radar_points[msg.track_ID].y_value =  msg.track_range*(sin (M_PI*msg.track_angle/180.0));
    ref_array[msg.track_ID] = true;
  }}

int main(int argc, char *argv[]) {
  ros::init(argc, argv,"radar_node");
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::NodeHandle nh;
  ros::Subscriber radar_sub= nh.subscribe("parsed_tx/radartrack" ,1000, RadarCallback);
  ros::init(argc, argv, "radar_viz");//publisher
  ros::Rate r(1);
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  visualization_msgs::Marker path;
  // --------------------Create Markers---------------------
  for (int i=0; i<64; i++)
    {
      ref_array[i] = false;
    }
  ros::Rate loop_rate(1000);
  while(ros::ok()) {
    ros::spinOnce();
    if(list_is_full) {
     
      //------Create Test Path Points-----
      test_path[0].x_coord = 1;
      test_path[1].x_coord = 3;
      test_path[2].x_coord = 5;
      test_path[3].x_coord = 7;

      test_path[0].y_coord = 1;
      test_path[1].y_coord = 3;
      test_path[2].y_coord = 4;
      test_path[3].y_coord = 8;

      for (int var = 0; var < (sizeof(test_path)/sizeof(test_path[0])); var++){
	printpoints(var,rtp = 1, marker_c= 2, dur = 5);
      }
      //Create Min distance filter here--------------------------------

      for (int var = 0; var< 64; var++){
	float min_dist1 = 100001.0;
	float min_dist2 = 10000.00;
	int path_id1 = 101;
	int path_id2 = 100;
	float line_dist = 0.0;
	float filter_dist = 5.0;
	float topeq = 0.0;
	float bottomeq = 0.0;
	float front_dist = 0.0;
	float side_dist = 0.0;
	if(ref_array[var] == true){
	  for (int t = 0; t < (sizeof(test_path)/sizeof(test_path[0])); t++){

	    float dist = sqrtf((pow(radar_points[var].x_value- test_path[t].x_coord,2.0))+(pow(radar_points[var].y_value-test_path[t].y_coord,2.0)));
	    dist1 = dist; 
	    if (dist <min_dist1){
	      min_dist2 = min_dist1;
	      min_dist1 = dist;
	      path_id2 = path_id1;
	      path_id1 = t;
	    }
	    else if(dist <min_dist2 && path_id1 != t){
	      min_dist2 = dist;
	      path_id2 = t;
	    }}
	  //Line Dist calculations:
	  topeq = radar_points[var].x_value*(test_path[path_id2].y_coord-test_path[path_id1].y_coord)-(radar_points[var].y_value*(test_path[path_id2].x_coord-test_path[path_id1].x_coord))+test_path[path_id2].x_coord*test_path[path_id1].y_coord-test_path[path_id2].y_coord*test_path[path_id1].x_coord;

	  bottomeq = (test_path[path_id2].y_coord - test_path[path_id1].y_coord)*(test_path[path_id2].y_coord - test_path[path_id1].y_coord) + (test_path[path_id2].x_coord - test_path[path_id1].x_coord)*(test_path[path_id2].x_coord - test_path[path_id1].x_coord);

	  line_dist = fabs((topeq)/(sqrtf(bottomeq)));
	  front_dist = radar_points[var].x_value - test_path[path_id1].x_coord;
	  side_dist = radar_points[var].y_value - test_path[path_id1].y_coord;
	  //------plot points-----
	  if(line_dist < filter_dist && front_dist < filter_dist && side_dist <filter_dist) {
	    printpoints(var,rtp = 2, marker_c= 3, dur = 0.1);
	    ref_array[var] = false;
	  }
	  else{
	    printpoints(var, rtp = 2, marker_c=1, dur= 0.1);
	    ref_array[var] = false;
	  }}}
      list_is_full = false;
    }
    loop_rate.sleep();
  }
  return 0;
}




