#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "testing/EsrTrack.h"


testing::EsrTrack my_esr_var;
testing::Esrtrack my_esr_array[64];


/*
 *
 * CALLBACK
 *
 *
 */
void RadarCallback(testing::EsrTrack msg) {
  std::cout << "received a msg!" << std::endl;
  printf("track ID: %d\n", msg.track_ID);
  printf("status ID: %d\n", msg.track_status);

  my_esr_array[msg.index - 1] = msg;
}



/*
 *
 * CHECK IF OBJECT IN PATH
 *
 *
 */
bool ObjectInPath(Path _path, RadarObject _radar_object, ) {

}


/*
 *
 * MAIN
 *
 *
 */
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "radar_node");
  ros::NodeHandle nh;
  ros::Subscriber radar_sub = nh.subscribe("parsed_tx/radartrack", 1, RadarCallback);

  ros::Rate loop_rate(10);

  while(ros::ok()) {

    my_esr_array[2].status;
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
