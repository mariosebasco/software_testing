/*
 * This script publishes the IMU data
 * Because of uncertainty in the magnetometer data I am going to average it with 
 * consecutive GPS readings
 *
 *
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
// #include "nav_msgs/Odometry.h"
// #include "testing/encoder_msg.h"

#include <stdio.h>
#include "vn100.h"
#include <math.h>
#include "testing/NovatelVelocity.h"

const char* const COM_PORT = "/dev/ttyS0";
const int BAUD_RATE = 115200;
VnVector3 GPS_vel;
bool should_start = false;

void GpsVelCB(testing::NovatelVelocity msg) {
  should_start = true;
  
  double hor_speed = msg.horizontal_speed;
  double track_ground = msg.track_ground;
  
  GPS_vel.c0 = hor_speed*cos(track_ground*M_PI/180);
  GPS_vel.c1 = hor_speed*sin(track_ground*M_PI/180);
  GPS_vel.c2 = 0.0;
}

int main(int argc, char **argv)
{

  //Init ROS
  ros::init(argc, argv, "imu_node");
	
  ros::NodeHandle n;
  ros::Subscriber gps_vel_sub = n.subscribe("bestvel", 1, GpsVelCB);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
  // ros::Subscriber odom_sub = n.subscribe("odom/gps_raw", 1, odom_callback);
  // ros::Subscriber encoder_sub = n.subscribe("encoder_counts", 1, encoder_callback);

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
  double imu_drift;
  
  char tmp_msg[50];

  ROS_INFO("connecting device\n");
  errorCode = vn100_connect(
			    &vn100,
			    COM_PORT,
			    BAUD_RATE);
  
  /* Make sure the user has permission to use the COM port. */
  if (errorCode == VNERR_PERMISSION_DENIED) {

    ROS_WARN("Current user does not have permission to open the COM port.\n");
    ROS_WARN("Try running again using 'sudo'.\n");

    return 0;
  }
  else if (errorCode != VNERR_NO_ERROR) {
    ROS_WARN("Error encountered when trying to connect to the sensor.\n");
    
    return 0;
  }
  ROS_INFO("Connected\n");

  //
  while (ros::ok())
  {

    ros::spinOnce();
    if(!should_start) continue;
    
    errorCode = vn100_setVelocityCompenstationMeasurement(&vn100, GPS_vel, true);
    if(errorCode != VNERR_NO_ERROR) ROS_INFO("Could not set vel compensation meas.\n");
    
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

    //convert imu axis to chassis axis
    Imu_data.orientation.x = 0.0;
    Imu_data.orientation.y = 0.0;
    Imu_data.orientation.z = quaternion.z;
    Imu_data.orientation.w = quaternion.w;

    n.getParam("/calibration_value", imu_drift);
    //printf("calibration value: %f\n", imu_drift);
    if(imu_drift != 0.0) {
      //average orientation obtained from magnetometer and consecutive gps points
      // double del_northing = gps_odom.pose.pose.position.x - prev_gps_odom.pose.pose.position.x;
      // double del_easting = gps_odom.pose.pose.position.y - prev_gps_odom.pose.pose.position.y;
      
      // if((encoder1_count == 0 && encoder2_count == 0) || encoder1_count == -encoder2_count) {
      // 	printf("no movement\n");
      // }
      // else {
      // 	double theta_gps = atan2(del_easting, del_northing);

      // 	tf::Quaternion imu_quat = tf::Quaternion(0.0, 0.0, quaternion.z, quaternion.w);
      // 	double theta_imu = imu_quat.getAngle();
      // 	if(theta_imu > M_PI) theta_imu -= 2*M_PI;
      
      // 	double theta_average = (theta_imu + theta_gps)/2.0;

      // 	tf::Quaternion averaged_quat = tf::createQuaternionFromRPY(0.0, 0.0, theta_average);

      // 	Imu_data.orientation.z = averaged_quat[2];
      // 	Imu_data.orientation.w = averaged_quat[3];
      
      // 	printf("theta_gps: %f\n", theta_gps);
      // 	printf("theta_imu: %f\n", theta_imu);
      // 	printf("theta_average: %f\n", theta_average);
      // }
      
      tf::Quaternion odom_quat = tf::Quaternion(0.0, 0.0, quaternion.z, quaternion.w);
      tf::Quaternion imu_quat = tf::createQuaternionFromRPY(0.0, 0.0, imu_drift);
      odom_quat = imu_quat*odom_quat;
      odom_quat.normalize();

      Imu_data.orientation.z = odom_quat[2];
      Imu_data.orientation.w = odom_quat[3];
      
      //printf("calibrated yaw: %f\n", getYaw(odom_quat));
    }

    
    Imu_data.linear_acceleration.x = acceleration.c0;
    Imu_data.linear_acceleration.y = acceleration.c1;
    Imu_data.linear_acceleration.z = 0.0;//acceleration.c2;

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
    ROS_WARN("Error encountered when trying to disconnect from the sensor.\n");
		
    return 0;
  }
  

  return 0;
}
