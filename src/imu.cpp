/*
 *
 *
 *
 *
 *
 */


#include "testing/talker.h"

const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;

int main(int argc, char **argv)
{
  //Init ROS
  ros::init(argc, argv, "talker");
	
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("chatter", 1000);

  ros::Rate loop_rate(10);

  sensor_msgs::Imu Imu_data;

  //Init IMU
  VN_ERROR_CODE errorCode;
  Vn100 vn100;

  VnVector3 acceleration;
  VnVector3 angular_vel;
  VnQuaternion quaternion;
  VnYpr ypr;

  char tmp_msg[50];

  printf("connecting device\n");
  errorCode = vn100_connect(
			    &vn100,
			    COM_PORT,
			    BAUD_RATE);
  
  /* Make sure the user has permission to use the COM port. */
  if (errorCode == VNERR_PERMISSION_DENIED) {

    printf("Current user does not have permission to open the COM port.\n");
    printf("Try running again using 'sudo'.\n");

    return 0;
  }
  else if (errorCode != VNERR_NO_ERROR) {
    printf("Error encountered when trying to connect to the sensor.\n");
    
    return 0;
  }
  printf("Connected\n");
  
  //
  while (ros::ok())
  {

    errorCode = vn100_getQuaternion(&vn100, &quaternion);
    errorCode = vn100_getAcceleration(&vn100, &acceleration);
    errorCode = vn100_getAngularRate(&vn100, &angular_vel);

    Imu_data.header.frame_id = "1";
    Imu_data.header.stamp = ros::Time::now();

    Imu_data.orientation.x = quaternion.x;
    Imu_data.orientation.y = quaternion.y;
    Imu_data.orientation.z = quaternion.z;
    Imu_data.orientation.w = quaternion.w;

    Imu_data.linear_acceleration.x = acceleration.c0;
    Imu_data.linear_acceleration.y = acceleration.c1;
    Imu_data.linear_acceleration.z = acceleration.c2;

    Imu_data.angular_velocity.x = angular_vel.c0;
    Imu_data.angular_velocity.y = angular_vel.c1;
    Imu_data.angular_velocity.z = angular_vel.c2;
    
    chatter_pub.publish(Imu_data);

    loop_rate.sleep();
  }

  errorCode = vn100_disconnect(&vn100);
	
  if (errorCode != VNERR_NO_ERROR) {
    printf("Error encountered when trying to disconnect from the sensor.\n");
		
    return 0;
  }
  

  return 0;
}
