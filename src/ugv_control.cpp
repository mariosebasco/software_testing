#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

class CmdVelClass {
public:
  CmdVelClass() {
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);    
    sub = nh.subscribe("joy", 1, &CmdVelClass::controllercb, this);
    linearGain = 0.55;
    angularGain = 0.55;
  }

  void controllercb( sensor_msgs::Joy msg) {
    output_vel.linear.x = linearGain*msg.axes[1];
    output_vel.angular.z = angularGain*msg.axes[3];

    pub.publish(output_vel);  
  }

private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  float linearGain;
  float angularGain;
  geometry_msgs::Twist output_vel;
};

//main
int main(int argc, char **argv) {
  ros::init(argc, argv, "ugv_control_node");
  CmdVelClass cmd_vel_object;
  
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

