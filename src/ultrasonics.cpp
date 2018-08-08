#include <iostream>
#include <math.h>
#include <string.h>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>

#define NUM_SONARS 1

struct Sonar {
  float x;
  float y;
  float theta;
  float fov;
};

float MAX_WHEEL_DECEL = 0.25; //m/s^2
float VEHICLE_WIDTH = 0.514;
float VEHICLE_LENGTH = 0.8;
bool should_start = false;
Sonar sonars[16];

tf::Quaternion odom_quat;
nav_msgs::Odometry odom;

float WrapAngle(float angle);
bool CheckIntersection(float _x_init, float _y_init, float _theta_init, float _x, float _y, float _theta, int _sonar_id, float _range);
void OdomCB(nav_msgs::Odometry msg);
void SonarCB(sensor_msgs::Range msg);


/************************************************************************************************
 *                                                                                              *
 *                                         MAIN                                                 *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ultrasonic_node");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("local/odom", 1, OdomCB);
  
  ros::Subscriber sonar_sub1 = nh.subscribe("sonar/as_tx/sensor1", 1, SonarCB);
  // ros::Subscriber sonar_sub2 = nh.subscribe("sonar/as_tx/sensor2", SonarCB);
  // ros::Subscriber sonar_sub3 = nh.subscribe("sonar/as_tx/sensor3", SonarCB);
  // ros::Subscriber sonar_sub4 = nh.subscribe("sonar/as_tx/sensor4", SonarCB);
  // ros::Subscriber sonar_sub5 = nh.subscribe("sonar/as_tx/sensor5", SonarCB);
  // ros::Subscriber sonar_sub6 = nh.subscribe("sonar/as_tx/sensor6", SonarCB);
  // ros::Subscriber sonar_sub7 = nh.subscribe("sonar/as_tx/sensor7", SonarCB);
  // ros::Subscriber sonar_sub8 = nh.subscribe("sonar/as_tx/sensor8", SonarCB);
  
  sonars[0].x = -VEHICLE_LENGTH/2.0;
  sonars[0].y = VEHICLE_WIDTH/2.0 - 0.105;
  sonars[0].theta = M_PI/2.0;
  sonars[0].fov = 1.05;
  
  ros::Rate loop_rate(100);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


/************************************************************************************************
 *                                                                                              *
 *                                   ODOMETRY CALLBACK FUNCTION                                 *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
void OdomCB(nav_msgs::Odometry msg) {
  should_start = true;
  odom = msg;
  odom_quat = tf::Quaternion(0.0, 0.0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
}



/************************************************************************************************
 *                                                                                              *
 *                                   SONAR CALLBACK FUNCTION                                    *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
void SonarCB(sensor_msgs::Range msg) {
  if(!should_start) return;

  if(msg.range > 1.2) return;

  char *pEnd;
  std::string my_str = msg.header.frame_id;
  char *char_sonar_id = &(my_str[my_str.size() - 1]);
  int sonar_id = atoi(char_sonar_id);

  sonars[sonar_id - 1].fov = msg.field_of_view;
  
  float curr_x = odom.pose.pose.position.x;
  float curr_y = odom.pose.pose.position.y;
  float curr_theta = getYaw(odom_quat);
  if(curr_theta < 0.0) curr_theta += 2.0*M_PI;
    
  //if maximum decceleration is applied, where will we stop
  float trans_vel = odom.twist.twist.linear.x;
  float rot_vel = odom.twist.twist.angular.z;
  // float trans_vel = 0.0;
  // float rot_vel = -0.5;
  
  float right_wheel_vel = fabs(trans_vel - rot_vel*VEHICLE_WIDTH/2.0);
  float left_wheel_vel = fabs(trans_vel + rot_vel*VEHICLE_WIDTH/2.0);
  float move_time;
  if(fabs(right_wheel_vel) >= fabs(left_wheel_vel)) move_time = fabs(right_wheel_vel/MAX_WHEEL_DECEL);
  else move_time = fabs(left_wheel_vel/MAX_WHEEL_DECEL);
  //float move_time = 5.0;
  float resolution = 0.1;
  int num_steps = int(move_time / resolution);
  
  for (int i = 0; i < num_steps; i++) {
  //for(int i = 30; i < 31; i++) {

    float next_x, next_y, next_theta;
    if(rot_vel == 0.0) {
      next_theta = curr_theta;
      next_x = curr_x + (trans_vel)*cos(curr_theta)*i*resolution;
      next_y = curr_y + (trans_vel)*sin(curr_theta)*i*resolution;
    }
    else {
      float radius = trans_vel / rot_vel;
      float circle_x = curr_x - radius*sin(curr_theta);
      float circle_y = curr_y + radius*cos(curr_theta);

      next_theta = WrapAngle(curr_theta + rot_vel*i*resolution);
      if(next_theta < 0.0) next_theta += 2.0*M_PI;
      next_x = circle_x + radius*sin(next_theta);
      next_y = circle_y - radius*cos(next_theta);
    }
  
    //check intersection between vehicle (after decceleration) and radar object
    bool collision = CheckIntersection(curr_x, curr_y, curr_theta, next_x, next_y, next_theta,
				       sonar_id, msg.range);
    if(collision) {
      ROS_WARN("SONAR %d COLLISION DETECTED IN %f SECONDS\n", sonar_id, i*resolution);
      return;
    }
  }
}



/************************************************************************************************
 *                                                                                              *
 *               CHECK IF VEHICLE HAS COLLIDED WITH ARC CREATED BY SONAR                        *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
bool CheckIntersection(float _x_init, float _y_init, float _theta_init, float _x, float _y, float _theta, int _sonar_id, float _range) {
  float x = _x;
  float y = _y;
  float theta = _theta;
  float x_init = _x_init;
  float y_init = _y_init;
  float theta_init = _theta_init;

  int sonar_id = _sonar_id;

  float r = _range;
  //float r = 0.05;
  float local_x = sonars[sonar_id - 1].x;
  float local_y = sonars[sonar_id - 1].y;

  float h = x_init + local_x*cos(theta_init) - local_y*sin(theta_init);
  float k = y_init + local_x*sin(theta_init) + local_y*cos(theta_init);

  // printf("********************\n");
  // printf("curr theta: %f\n", theta_init*180.0/M_PI);
  // printf("next theta: %f\n", theta*180.0/M_PI);
  // printf("curr_x: %f\n", x_init);
  // printf("next_x: %f\n", x);
  // printf("curr_y: %f\n", y_init);
  // printf("next_y: %f\n", y);
  //printf("radius: %f\n", r);
  // printf("local_x: %f\n", local_x);
  // printf("local_y: %f\n", local_y);
  // printf("h: %f\n", h);
  // printf("k: %f\n", k);
  // printf("********************\n");


  //intersection has occured if arc is completely within rectangle
  //or if the arc intersects with one of the rectangle edges

  //First we check if any of the edges of the rectangle intersect with the arc
  float a, b, c, A, B, C, discriminant, xsol1, xsol2, ysol1, ysol2;
  float u1, u2, px, py;
  float x_edges[5] = {-VEHICLE_LENGTH/2.0, -VEHICLE_LENGTH/2.0, VEHICLE_LENGTH/2.0, VEHICLE_LENGTH/2.0, -VEHICLE_LENGTH/2.0};
  float y_edges[5] = {VEHICLE_WIDTH/2.0, -VEHICLE_WIDTH/2.0, -VEHICLE_WIDTH/2.0, VEHICLE_WIDTH/2.0, VEHICLE_WIDTH/2.0};
  for (int i = 0; i < 4; i++) {
    float x1 = x + x_edges[i]*cos(theta) - y_edges[i]*sin(theta);
    float y1 = y + x_edges[i]*sin(theta) + y_edges[i]*cos(theta);
    float x2 = x + x_edges[i+1]*cos(theta) - y_edges[i+1]*sin(theta);
    float y2 = y + x_edges[i+1]*sin(theta) + y_edges[i+1]*cos(theta);

    a = y2 - y1;
    b = -(x2 - x1);
    c = (x2*y1 - y2*x1);

    //printf("%d\n", i);
    // printf("a: %f\n", a);
    // printf("b: %f\n", b);
    // printf("c: %f\n", c);
    // printf("y1: %f\n", y1);
    // printf("y2: %f\n", y2);
    // printf("x1: %f\n", x1);
    // printf("x2: %f\n", x2);
    

    if(a == 0.0) { //special case
      if(y2 >= (k - r) && y2 <= (k + r)) {
	A = 1.0;
	B = 2.0*h;
	C = h*h + pow((y2 - k), 2) - r*r;

	discriminant = B*B - 4.0*A*C;
	if(discriminant < 0.0) continue; //shouldn't happen, but just in case
	xsol1 = (-B + sqrt(discriminant))/(2.0*A);
	xsol2 = (-B - sqrt(discriminant))/(2.0*A);
	ysol1 = y2;
	ysol2 = y2;
      }
      else continue;
    }
    
    else {
      A = pow(a, 2) + pow(b, 2);
      B = 2.0*c*b + 2.0*b*a*h - 2.0*a*a*k;
      C = c*c + 2.0*c*a*h + a*a*h*h + a*a*k*k - a*a*r*r;
      discriminant = B*B - 4.0*A*C;
      
      if(discriminant < 0.0) continue;
      //printf("discr: %f\n", discriminant);
      ysol1 = (-B + sqrt(discriminant))/(2.0*A);
      ysol2 = (-B - sqrt(discriminant))/(2.0*A);
      xsol1 = (-b*ysol1 - c)/a;
      xsol2 = (-b*ysol2 - c)/a;
    }
    
    px = x2 - x1;
    py = y2 - y1;
    u1 = ((xsol1 - x1)*px + (ysol1 - y1)*py)/(pow(px, 2) + pow(py, 2));
    u2 = ((xsol2 - x1)*px + (ysol2 - y1)*py)/(pow(px, 2) + pow(py, 2));
    
    // printf("x1 sol: %f\n", xsol1);
    // printf("y1 sol: %f\n", ysol1);
    // printf("x2 sol: %f\n", xsol2);
    // printf("y2 sol: %f\n", ysol2);

    // printf("u1: %f\n", u1);
    // printf("u2: %f\n", u2);

    bool u1_is_solution = false;
    bool u2_is_solution = false;
    if((u1 >= 0.0) && (u1 <= 1.0)) u1_is_solution = true;
    if((u2 >= 0.0) && (u2 <= 1.0)) u2_is_solution = true;
    if(!u1_is_solution && !u2_is_solution) continue;
      
    float sol1_theta = atan2(ysol1 - k, xsol1 - h);
    float sol2_theta = atan2(ysol2 - k, xsol2 - h);
    // float sol1_theta = -atan2(xsol1 - h, ysol1 - k);
    // float sol2_theta = -atan2(xsol2 - h, ysol2 - k);
    if(sol1_theta < 0.0) sol1_theta += 2.0*M_PI;
    if(sol2_theta < 0.0) sol2_theta += 2.0*M_PI;
    // sol1_theta = WrapAngle(sol1_theta + theta_init + sonars[sonar_id - 1].theta);
    // sol2_theta = WrapAngle(sol2_theta + theta_init + sonars[sonar_id - 1].theta);

    // printf("sol1 theta: %f\n", sol1_theta*180.0/M_PI);
    // printf("sol2 theta: %f\n", sol2_theta*180.0/M_PI);
    
    float sonar_theta = WrapAngle(theta_init + sonars[sonar_id - 1].theta);
    //float sonar_theta = WrapAngle(sonars[sonar_id - 1].theta);
    float sonar_theta_lo = WrapAngle(sonar_theta - sonars[sonar_id - 1].fov/2.0);
    float sonar_theta_hi = WrapAngle(sonar_theta + sonars[sonar_id - 1].fov/2.0);

    //printf("sonar theta: %f\n", sonar_theta*180.0/M_PI);
    // printf("sonar theta lo: %f\n", sonar_theta_lo*180.0/M_PI);
    // printf("sonar theta hi: %f\n", sonar_theta_hi*180.0/M_PI);

    
    if(sonar_theta_hi > sonar_theta_lo) {
      if(sol1_theta > sonar_theta_lo && sol1_theta < sonar_theta_hi && u1_is_solution) return true; //arc intersected line segment
      if(sol2_theta > sonar_theta_lo && sol2_theta < sonar_theta_hi && u2_is_solution) return true;
    }
    else {
      if((sol1_theta <= M_PI) && u1_is_solution) {if(sol1_theta < sonar_theta_hi) return true;} //arc intersected line segment
      if((sol2_theta <= M_PI) && u2_is_solution) {if(sol2_theta < sonar_theta_hi) return true;}
      if((sol1_theta >= M_PI) && u1_is_solution) {if(sol1_theta > sonar_theta_lo) return true;}
      if((sol2_theta >= M_PI) && u2_is_solution) {if(sol2_theta > sonar_theta_lo) return true;}
    }
  }

  //now if not, we check if the circle is completely inside the rectangle
  // if((2.0*r > VEHICLE_LENGTH) || (2.0*r > VEHICLE_WIDTH)) return false;
  // printf("ARC IN RECT\n");
  // float x_top = x + x_edges[3]*cos(theta);
  // float y_top = y + x_edges[3]*sin(theta);
  // float x_right = x - y_edges[3]*sin(theta);
  // float y_right = y + y_edges[3]*cos(theta);

  // float dist_from_vert = fabs((y_top - y)*h - (x_top - x)*k + x_top*y - y_top*x)/sqrt(pow(y_top - y, 2) + pow(x_top - x, 2));
  // float dist_from_hor = fabs((y_right - y)*h - (x_right - x)*k + x_right*y - y_right*x)/sqrt(pow(y_right - y, 2) + pow(x_right - x, 2));

  // if((dist_from_vert + r) < VEHICLE_WIDTH/2.0 && (dist_from_hor + r) < VEHICLE_LENGTH/2.0) return true; //circle is completely within rectangle

  return false;
}

/************************************************************************************************
 *                                                                                              *
 *                                   WRAP ANGLE -- HELPER FUNCTION                              *
 *                                                                                              *
 *                                                                                              *
 ***********************************************************************************************/
float WrapAngle( float angle ) {
  float twoPi = 2.0 * M_PI;
  return angle - twoPi * floor( angle / twoPi );
}
