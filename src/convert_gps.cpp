/*
 *
 * Class KmlExport -- takes kml file and creates new file with only lat and long
 * Class PubCoods -- takes the previously made file and publishes it to be converted to UTM coods
 *
 *
 */

#include <iostream>
#include <fstream>
#include <string>
#include <typeinfo>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "utm.cpp"

/***********************************************************************
 *                                                                     *
 *                      KML Export Class                                *
 *                                                                     *
 *********************************************************************/
class KmlExport {
public:
  KmlExport() {}
  
  void ExportFile(char *inputFile, char *outputFile) {
    std::string line;
    std::string latitude, longitude;
    std::string coods_tag = "<coordinates>";
    std::string coods_tag_close = "</coordinates>";
    unsigned int open_char, close_char, length, cood_tag_open, cood_tag_length;
    double lat_num, long_num, northing, easting;
    double prev_northing, prev_easting, next_northing, next_easting;
      
    int e_id, zone;

    open_char = 0;
    close_char = 0;
    length = 0;
    e_id = 22;

    inFile.open(inputFile);
    outFile.open(outputFile);

    if (!inFile) {
      printf("Unable to open file\n");
    }
    if (!outFile) {
      printf("Unable to open file\n");
    }

    std::getline(inFile, line);
    open_char = line.find_first_of("<");
    close_char = line.find_first_of(">");
    length = close_char - open_char + 1;
    while(line.compare(open_char, length, coods_tag) != 0) {
      open_char = line.find_first_of("<");
      close_char = line.find_first_of(">");
      length = close_char - open_char + 1;
      std::getline(inFile, line);
    } //find the coordinates tag

    cood_tag_open = open_char;
    cood_tag_length = length + 1;

    std::getline(inFile, line);

    while(line.compare(cood_tag_open, cood_tag_length, coods_tag_close) != 0) {
      //find and convert the longitude and latitude to UTM
      open_char = line.find_first_not_of(" ");
      close_char = line.find_first_of(",");
      length = close_char - open_char;
      longitude  = line.substr (open_char, length);
      //std::cout <<  longitude << std::endl;

      open_char = line.find_first_of(",") + 1;
      close_char = line.find_last_of(",");
      length = close_char - open_char;
      latitude  = line.substr (open_char, length);
      //std::cout <<  latitude << std::endl;

      //convert to floats
      char *pEnd;
      long_num = strtof(longitude.c_str(), &pEnd);      
      lat_num = strtof(latitude.c_str(), &pEnd);      

      LLtoUTM(e_id, lat_num, long_num, northing, easting, zone);

      //interpolate data and write to file
      //interpolatePoints(northing, easting);
      outFile << std::fixed << northing << std::endl;
      outFile << std::fixed << easting << std::endl;

      std::getline(inFile, line);
    }
    inFile.close();
    outFile.close();
  }


  void ExportVelFile( char *_velFile, char *_coodFile) {
    double prev_northing, prev_easting, curr_northing, curr_easting, next_northing, next_easting;
    float curr_max_vel, abs_max_vel, abs_min_vel;
    std::ofstream velFile;
    std::ifstream coodFile;
    std::string line;
    char* pEnd;

    abs_max_vel = 1.0;
    abs_min_vel = 0.5;
    
    velFile.open(_velFile);
    coodFile.open(_coodFile);

    if (!velFile) {
      printf("Unable to open file\n");
    }
    if (!coodFile) {
      printf("Unable to open file\n");
    }

    //first point
    std::getline(coodFile, line);
    prev_northing = strtof(line.c_str(), &pEnd);
    std::getline(coodFile, line);
    prev_easting = strtof(line.c_str(), &pEnd);

    //second point
    std::getline(coodFile, line);
    curr_northing = strtof(line.c_str(), &pEnd);
    std::getline(coodFile, line);
    curr_easting = strtof(line.c_str(), &pEnd);
    
    velFile << std::fixed << abs_min_vel << std::endl;
    
    while(ros::ok()) {
      std::getline(coodFile, line);
      if(coodFile.eof()) break;
      next_northing = strtof(line.c_str(), &pEnd);
      std::getline(coodFile, line);
      next_easting = strtof(line.c_str(), &pEnd);      
      // double x1, x2, y1, y2, b, c, a, x_line, y_line, dist_to_line, dist_to_point;
      // float theta;
      // x1 = prev_northing;
      // y1 = prev_easting;
      // x2 = next_northing;
      // y2 = next_easting;
      // a = (y2 - y1); // line between these two markers
      // b = -(x2 - x1);
      // c = x2*y1 - x1*y2;
      // dist_to_line = fabs(a*curr_northing + b*curr_easting + c)/sqrt(pow(a, 2) + pow(b, 2));
      // x_line = (b*(b*curr_northing - a*curr_easting) - a*c)/(pow(a, 2) + pow(b, 2)); 
      // y_line = (a*(-b*curr_northing + a*curr_easting) - b*c)/(pow(a, 2) + pow(b, 2));
      // dist_to_point = sqrt(pow(prev_easting - y_line, 2) + pow(prev_northing - x_line, 2));
      // theta = atan2(dist_to_line, dist_to_point)*180.0/M_PI;
      float x1 = prev_northing;
      float x2 = curr_northing;
      float x3 = next_northing;
      float y1 = prev_easting;
      float y2 = curr_easting;
      float y3 = next_easting;
      
      float a = sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2));
      float b = sqrt(pow(y2 - y3, 2) + pow(x2 - x3, 2));
      float c = sqrt(pow(y1 - y3, 2) + pow(x1 - x3, 2));

      float theta = acos((pow(c, 2) - pow(a, 2) - pow(b, 2)) / (2*a*b));

      if(theta > M_PI/2.0) curr_max_vel = abs_min_vel;
      else curr_max_vel = abs_min_vel + (abs_max_vel - abs_min_vel)*((M_PI/2.0) - theta)/(M_PI/2.0);

      std::cout << theta*180.0/M_PI << std::endl;
      std::cout << curr_max_vel << std::endl;
      
      velFile << std::fixed << curr_max_vel << std::endl;

      prev_northing = curr_northing;
      prev_easting = curr_easting;
      curr_northing = next_northing;
      curr_easting = next_easting;
    }
    velFile << std::fixed << abs_min_vel << std::endl;
    velFile.close();
    coodFile.close();
  }
  
private:
  std::ifstream inFile;
  std::ofstream outFile;
  int line_count;

  void interpolatePoints(double northing, double easting) {
    static double prev_northing = 0.0;
    static double prev_easting = 0.0;
    float increment;
    int num_incs;

    if(line_count == 1) {//fix code *********
      std::cout << "received one point" << std::endl;
      outFile << std::fixed << northing << std::endl;
      outFile << std::fixed << easting << std::endl;
    }
    else {//interpolate if points are far apart -- We want a point every meter
      std::cout << "received another point" << std::endl;
      num_incs = (abs(northing - prev_northing) > abs(easting - prev_easting)) ? ceil(int(abs(northing - prev_northing))) : ceil(int(abs(easting - prev_easting)));
      increment = 1.0/num_incs;
      for(int i = 1; i <= num_incs; i++) {
	outFile << std::fixed << prev_northing + i*increment*(northing - prev_northing) << std::endl;
	outFile << std::fixed << prev_easting + i*increment*(easting - prev_easting) << std::endl;
      }
    }
    
    prev_northing = northing;
    prev_easting = easting;
  }
};





KmlExport *kmlObject;

/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_node");
  
  char inputFile[] = "/home/robot/catkin_ws/src/testing/gps_files/gps_raw.kml";
  char outputFile[] = "/home/robot/catkin_ws/src/testing/gps_files/path.txt";
  char velFile[] = "/home/robot/catkin_ws/src/testing/gps_files/vel.txt";
  kmlObject = new KmlExport();
  
  kmlObject->ExportFile(inputFile, outputFile);
  kmlObject->ExportVelFile(velFile, outputFile);

  std::cout << "UTM CONVERSION COMPLETE" << std::endl;

  return 0;
}



