/*
 *
 * Creates node to control UGV - Depending on parameter input can be controlled by
 * cmd_vel topic or xbox360 controller
 * This node is also the node hat publishes encoder counts
 */

#include <math.h>
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "rosgraph_msgs/Log.h"

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "testing/encoder_msg.h"

bool CONTROLLER_CONNECTED = true;

class RoboteqDriver {
public:

  sensor_msgs::Joy controller_state;
  geometry_msgs::Twist cmd_vel;
  int RPM;
  bool use_controller, should_start;
  int KP, KI;

  /***********************************************************************
   *                                                                     *
   *                      CONSTRUCTOR                                    *
   *                                                                     *
   *********************************************************************/
  RoboteqDriver(int rpm_input, bool controller_input) {
    encoder_pub = nh.advertise<testing::encoder_msg>("encoder_counts", 1);
    RPM = rpm_input;
    use_controller = controller_input;
    should_start = false;
    KP = 5;
    KI = 20;
  }


  /***********************************************************************
   *                                                                     *
   *                      INITIALIZE MOTORS                              *
   *                                                                     *
   *********************************************************************/
  void roboteqInit() {
    //Initialize motor driver
    status = device.Connect("/dev/ttyACM0");

    if(status != RQ_SUCCESS)
      {
	std::cout<<"Error connecting to device: "<<status<<"."<<std::endl;
      }
    else {
      std::cout<<"succeeded" <<std::endl;
    }

    //Make sure emergency stop is released
    if((status = device.SetCommand(_MG, 1)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }

    //MOTOR 1 CONFIG
    //*****************************************************************
    ROS_INFO("set encoder PPR and mode to closed loop");
    //Set encoder mode and PPR
    if((status = device.SetConfig(_EMOD, 1, 18)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
    if((status = device.SetConfig(_EPPR, 1, 2048)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);

    ROS_INFO("Set PID gain and motor closed loop mode");
    //set to closed loop mode and tune PID
    if((status = device.SetConfig(_MMOD, 1, 1)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
  
    //PID motor 1
    if((status = device.SetConfig(_KD, 1, 0)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
    if((status = device.SetConfig(_KI, 1, KI)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
    if((status = device.SetConfig(_KP, 1, KP)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);

    ROS_INFO("set max RPM");
    //max rpm
    if((status = device.SetConfig(_MXRPM, 1, RPM)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
    //***********************************************************

    //MOTOR 2 CONFIG
    //************************************************************
    //Set encoder mode and PPR
    if((status = device.SetConfig(_EMOD, 2, 34)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    
    //Wait 10 ms before sending another command to device
    usleep(10000);
    if((status = device.SetConfig(_EPPR, 2, 2048)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    
    //Wait 10 ms before sending another command to device
    usleep(10000);

    //set to closed loop mode and tune PID
    if((status = device.SetConfig(_MMOD, 2, 1)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    
    //Wait 10 ms before sending another command to device
    usleep(10000);
  
    //PID motor 1
    if((status = device.SetConfig(_KD, 2, 0)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    
    //Wait 10 ms before sending another command to device
    usleep(10000);
    
    if((status = device.SetConfig(_KI, 2, KI)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);
    
    if((status = device.SetConfig(_KP, 2, KP)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);

    //max rpm
    if((status = device.SetConfig(_MXRPM, 2, RPM)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    //Wait 10 ms before sending another command to device
    usleep(10000);

    ROS_INFO("motors configured");
    //********************************************************************
    
    joy_sub = nh.subscribe("joy", 1, &RoboteqDriver::controllerCb, this);
    vel_sub = nh.subscribe("cmd_vel", 1, &RoboteqDriver::velocityCb, this);
    rosout_sub = nh.subscribe("rosout_agg", 1, &RoboteqDriver::rosoutCB, this);
  }

  /***********************************************************************
   *                                                                     *
   *                      ROSOUT MESSAGES                                *
   *         CHECK IF CONTROLLER HAS BEEN DISCONNECTED                   *
   *                                                                     *
   *********************************************************************/
  void rosoutCB(const rosgraph_msgs::Log &msg) {
    if(msg.name == "/joy_node") {
      if(msg.function == "main" && (msg.line == 174 || msg.line == 316)) {
	CONTROLLER_CONNECTED = false;
      }
      else if(msg.function == "main" && msg.line == 181) {
	CONTROLLER_CONNECTED = true;
      }
    }
  }

  /***********************************************************************
   *                                                                     *
   *                      VELOCITY CALLBACK                              *
   *                                                                     *
   *********************************************************************/
  void velocityCb(const geometry_msgs::Twist &msg) {
    if(!use_controller) {
      should_start = true;
    }
    cmd_vel = msg;
  }

  /***********************************************************************
   *                                                                     *
   *                      CONTROLLER CALLBACK                            *
   *                                                                     *
   *********************************************************************/
  void controllerCb(const sensor_msgs::Joy &msg) {
    if(use_controller) {
      should_start = true;
    }

    if(msg.buttons[0] || msg.buttons[1] || msg.buttons[2] || msg.buttons[3]) {
      //EMERGENCY STOP
      if((status = device.SetCommand(_EX, 1)) != RQ_SUCCESS) {
	std::cout<<"failed --> "<<status<<std::endl;
      }
    }
    else if(msg.buttons[4]) { //left bumper
      use_controller = 1;
    }
    else if(msg.buttons[5]) { //right bumper
      use_controller = 0;
    }
    else {
      controller_state = msg;
    }
  }

  /***********************************************************************
   *                                                                     *
   *                      PUBLISH ENCODER COUNT                          *
   *                                                                     *
   *********************************************************************/
  void pub_encoder(int encoder1_count, int encoder2_count) {
    encoderCount.encoder1_count = encoder1_count;
    encoderCount.encoder2_count = encoder2_count;
    encoder_pub.publish(encoderCount);
  }

  /***********************************************************************
   *                                                                     *
   *                      RETURN BATTERY VOLTAGE                         *
   *                                                                     *
   *********************************************************************/
  int battVoltStatus() {
    int batt_volts;
    if((status = device.GetValue(_V, 2, batt_volts)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    if (batt_volts < 240) {
      ROS_WARN("WARNING: battery below 24V");
    }
    return batt_volts;
  }

  /***********************************************************************
   *                                                                     *
   *                      READ ENCODER COUNT                             *
   *                                                                     *
   *********************************************************************/
  int readEncoder(int encoder) {
    int encoder_count;
    if((status = device.GetValue(_CR, encoder, encoder_count)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
    return encoder_count;
  }


  /***********************************************************************
   *                                                                     *
   *                      SET MOTOR VELOCITY                             *
   *                                                                     *
   *********************************************************************/
  void setVelocity(int motor, int velocity) {
    if((status = device.SetCommand(_G, motor, velocity)) != RQ_SUCCESS) {
      std::cout<<"failed --> "<<status<<std::endl;
    }
  }

  
private:
  RoboteqDevice device;
  ros::NodeHandle nh;
  ros::Publisher encoder_pub;
  ros::Subscriber joy_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber rosout_sub;
  
  testing::encoder_msg encoderCount;
  int status;

};


/****************************************************************************************
 *                                                                                      *
 *                                                                                      *
 *                                    MAIN                                              *
 *                                                                                      *
 *                                                                                      *
 ***************************************************************************************/
int main(int argc, char *argv[]) {

  ros::init(argc, argv, "roboteq_node");

  bool use_controller;
  int RPM;
  float max_vel, wheel_radius, L;
  ros::param::get("/use_controller", use_controller);
  ros::param::get("/max_velocity", max_vel);
  ros::param::get("/wheel_radius", wheel_radius);
  ros::param::get("/vehicle_width", L);
    
  RPM = int(max_vel/wheel_radius*60/(2*M_PI));
  
  RoboteqDriver roboteq_object(RPM, use_controller);
  roboteq_object.roboteqInit();

  //variables
  int encoder1_count, encoder2_count, encoder1_init, encoder2_init, vel_motor1, vel_motor2;
  int batt_volts;
  float linear_gain = 1;
  float angular_gain = 0.75;

  batt_volts = roboteq_object.battVoltStatus();
  ROS_INFO("battery voltage: %f", float(batt_volts)/10.0);

  ros::Rate loop_rate(10);
  while(ros::ok()) {
    ros::spinOnce();

    //get *relative* encoder readings
    encoder1_count = -roboteq_object.readEncoder(1);
    encoder2_count = roboteq_object.readEncoder(2);
    
    //publish encoder count
    roboteq_object.pub_encoder(encoder1_count, encoder2_count);

    if(roboteq_object.should_start) {
      if(!CONTROLLER_CONNECTED) {
	roboteq_object.setVelocity(1,0);
	roboteq_object.setVelocity(2,0);
	continue;
      }
      
      //calculate speeds
      if (roboteq_object.use_controller) { //convert joystick input into velocity
	vel_motor1 = -int((roboteq_object.controller_state.axes[1]*linear_gain - roboteq_object.controller_state.axes[3]*angular_gain)*1000.0);
	vel_motor2 = int((roboteq_object.controller_state.axes[1]*linear_gain + roboteq_object.controller_state.axes[3]*angular_gain)*1000.0);

	//dead band
	vel_motor1 = abs(vel_motor1) < 20 ? 0 : vel_motor1;
	vel_motor2 = abs(vel_motor2) < 20 ? 0 : vel_motor2;

	//set velocities
	roboteq_object.setVelocity(1,vel_motor1);
	roboteq_object.setVelocity(2,vel_motor2);
      }
      else { //convert cmd_vel data to RPM ratio
	//convert m/s to RPM
	vel_motor1 = -int(((roboteq_object.cmd_vel.linear.x + L*roboteq_object.cmd_vel.angular.z/2.0)*60.0/(2.0*M_PI*wheel_radius))*1000.0/RPM);
	vel_motor2 = int(((roboteq_object.cmd_vel.linear.x - L*roboteq_object.cmd_vel.angular.z/2.0)*60.0/(2.0*M_PI*wheel_radius))*1000.0/RPM);
      
	//Keep within bounds
	vel_motor1 = vel_motor1 > 1000 ? 1000 : vel_motor1;
	vel_motor2 = vel_motor2 > 1000 ? 1000 : vel_motor2;
	vel_motor1 = vel_motor1 < -1000 ? -1000 : vel_motor1;
	vel_motor2 = vel_motor2 < -1000 ? -1000 : vel_motor2;

	//set velocities
	roboteq_object.setVelocity(1,vel_motor1);
	roboteq_object.setVelocity(2,vel_motor2);
      }
    }
    batt_volts = roboteq_object.battVoltStatus();

    loop_rate.sleep();

  }
  
  return 0;
}
