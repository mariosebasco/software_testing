/*
 *
 * Subscribes to cmd_vel topic
 * controls ugv motor speeds accordingly
 * subscribes to controller - kill is 'B' button is pressed
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/joy.h"

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"


class UgvController {
public:
  UgvController() {
    vel_sub = nh.subscribe("cmd_vel", 1, &UgvController::velCallback, this);
    joy_sub = nh.subscribe("joy", 1, &UgvController::joyCallback, this);
  }

  void velCallback() {
  }

  void joyCallback() {
  }
  

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber vel_sub;
  ros::Subscriber joy_sub;
};

int main(int argv, char *argv[]) {
  ros::init(argc, argv, "ugv_controller_node");
  UgvController controller_object;

  ros::Rate loop_rate(50);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
