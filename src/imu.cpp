/*
 *
 *
 *
 *
 *
 */
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"

#include <stdio.h>
#include "vn100.h"
#include <math.h>

const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;

int main(int argc, char **argv)
{

  //Init ROS
  ros::init(argc, argv, "imu_node");
	
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);

  ros::Rate loop_rate(10);

  sensor_msgs::Imu Imu_data;
  
  //Init IMU
  VN_ERROR_CODE errorCode;
  Vn100 vn100;

  VnVector3 acceleration;
  VnVector3 angular_vel;
  VnQuaternion quaternion;
  VnYpr ypr;

  double angularWalkVariance;
  VnVector3 angularRateVariance, magneticVariance, accelerationVariance;
  
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

    //testing for noise cancellation
    acceleration.c0 = abs(acceleration.c0) < 0.5 ? 0 : acceleration.c0;
    acceleration.c1 = abs(acceleration.c1) < 0.5 ? 0 : acceleration.c1;
    acceleration.c2 = abs(acceleration.c2) < 0.5 ? 0 : acceleration.c2;
    
    //variance
    errorCode = vn100_getFilterMeasurementVarianceParameters(&vn100,
							       &angularWalkVariance,
							       &angularRateVariance,
							       &magneticVariance,
							       &accelerationVariance);

    Imu_data.header.stamp = ros::Time::now();
    Imu_data.header.frame_id = "chassis";

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

    Imu_data.orientation_covariance[0] = magneticVariance.c0;
    Imu_data.orientation_covariance[4] = magneticVariance.c1;
    Imu_data.orientation_covariance[8] = magneticVariance.c2;
        
    Imu_data.angular_velocity_covariance[0] = angularRateVariance.c0;
    Imu_data.angular_velocity_covariance[4] = angularRateVariance.c1;
    Imu_data.angular_velocity_covariance[8] = angularRateVariance.c2;
    
    Imu_data.linear_acceleration_covariance[0] = accelerationVariance.c0;
    Imu_data.linear_acceleration_covariance[4] = accelerationVariance.c1;
    Imu_data.linear_acceleration_covariance[8] = accelerationVariance.c1;
    
    imu_pub.publish(Imu_data);

    loop_rate.sleep();
    
  }

  errorCode = vn100_disconnect(&vn100);
	
  if (errorCode != VNERR_NO_ERROR) {
    printf("Error encountered when trying to disconnect from the sensor.\n");
		
    return 0;
  }
  

  return 0;
}
