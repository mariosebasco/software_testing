#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "testing/encoder_msg.h"

sensor_msgs::Joy controller_state;
int should_start = 0;

void controller_cb(sensor_msgs::Joy msg) {
  controller_state = msg;
  should_start = 1;
}



int main(int argc, char *argv[])
{
  //initialize ROS
  ros::init(argc, argv, "motor_driver_node");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joy", 1, controller_cb);
  ros::Publisher encoder_pub;
  encoder_pub = nh.advertise<testing::encoder_msg>("encoder_counts", 1);

  testing::encoder_msg encoderCount;
  
  //Initialize motor driver
  RoboteqDevice device;
  int status = device.Connect("/dev/ttyACM0");

  if(status != RQ_SUCCESS)
    {
      std::cout<<"Error connecting to device: "<<status<<"."<<std::endl;
      return 1;
    }
  else {
    std::cout<<"succeeded" <<std::endl;
  }

  //variables
  int encoder1_count, encoder2_count, vel_motor1, vel_motor2;
  float gain = 0.75;

  // set encoder1 count to 0
  std::cout<<"- SetCommand(_C, 1, 0)...";
  if((status = device.SetCommand(_C, 1, 0)) != RQ_SUCCESS) {
    std::cout<<"failed --> "<<status<<std::endl;
  }
  else {
    std::cout<<"encoder1 set to 0"<<std::endl;
  }

  // set encoder2 count to 0
  std::cout<<"- SetCommand(_C, 2, 0)...";
  if((status = device.SetCommand(_C, 2, 0)) != RQ_SUCCESS) {
    std::cout<<"failed --> "<<status<<std::endl;
  }
  else {
    std::cout<<"encoder 2 set to 0"<<std::endl;
  }

  //Wait 10 ms before sending another command to device
  usleep(10000);

  ros::Rate loop_rate(10);
  while(ros::ok()) {

    //get encoder 1 count
    if((status = device.GetValue(_C, 1, encoder1_count)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    encoder1_count = -encoder1_count;

    //Wait 10 ms before sending another command to device
    usleep(10000);
  
    //get encoder 2 count
    if((status = device.GetValue(_C, 2, encoder2_count)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }

    //publish encoder count
    encoderCount.encoder1_count = encoder1_count;
    encoderCount.encoder2_count = encoder2_count;
    encoder_pub.publish(encoderCount);
    
    //Wait 10 ms before sending another command to device
    usleep(10000);
    
    //calculate speeds
    if (should_start) {
      vel_motor1 = int((controller_state.axes[1] - controller_state.axes[3])*gain*1000.0);
      vel_motor2 = -int((controller_state.axes[1] + controller_state.axes[3])*gain*1000.0);

      //set vel for motor 1
      if((status = device.SetCommand(_G, 1, vel_motor1)) != RQ_SUCCESS) {
	std::cout<<"failed --> "<<status<<std::endl;
      }

      // //Wait 10 ms before sending another command to device
      // usleep(10000);
    
      //set vel for motor 2
      if((status = device.SetCommand(_G, 2, vel_motor2)) != RQ_SUCCESS) {
	std::cout<<"failed --> "<<status<<std::endl;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }    

  device.Disconnect();
  return 0;
}
