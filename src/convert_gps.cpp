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

    while(line.compare(open_char, length, coods_tag) != 0) {
      std::getline(inFile, line);
      open_char = line.find_first_of("<");
      close_char = line.find_first_of(">");
      length = close_char - open_char + 1;
    } //find the coordinates tag
    cood_tag_open = open_char;
    cood_tag_length = length + 1;

    std::getline(inFile, line);
    line_count = 1;
    
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
      InterpolatePoints(northing, easting);
            
      std::getline(inFile, line);
      line_count += 1;
    }
    inFile.close();
    outFile.close();
  }

private:
  std::ifstream inFile;
  std::ofstream outFile;
  int line_count;

  void InterpolatePoints(double northing, double easting) {
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

class VelMap {
public:
  VelMap() {}

  void ExportFile(char *inputFile, char *outputFile) {
    std::string str_northing, str_easting;
    float temp_vel, line_slope, line_intercept;

    float distance_sum = 0.0;
    float start_vel = 0.5;
    int num_points_in_line = 5; //always odd
    float vel_array[num_points_in_line];
    //float distance_array[num_points_in_line - 2];
    float northing_array[num_points_in_line];
    float easting_array[num_points_in_line];

    char *pEnd;
    
    inFile.open(inputFile);
    outFile.open(outputFile);
    if (!inFile) {
      printf("Unable to open file\n");
    }
    if (!outFile) {
      printf("Unable to open file\n");
    }

    //Initialize
    for(int i = 0; i < num_points_in_line/2; i++) {
      outFile << start_vel << std::endl;
    }
    
    for(int i = 0; i < num_points_in_line; i++) {
      std::getline(inFile, str_northing);
      std::getline(inFile, str_easting);
      northing_array[i] = strtof(str_northing.c_str(), &pEnd);
      easting_array[i] = strtof(str_easting.c_str(), &pEnd);
    }

    while(true) {
      line_slope = (easting_array[0] - easting_array[num_points_in_line - 1]) / \
	(northing_array[0] - northing_array[num_points_in_line - 1]);
      line_intercept = easting_array[0] - line_slope*northing_array[0];

      for(int i = 0; i < (num_points_in_line - 2); i++) {
	distance_sum += fabs(line_slope*northing_array[i+1] - easting_array[i+1] + line_intercept)\
	  / sqrt(pow(line_slope, 2) + 1);
      }
      outFile << FindMaxVelocity(distance_sum) << std::endl;
      distance_sum = 0.0;

      for(int i = 0; i < (num_points_in_line - 1); i++) {
	northing_array[i] = northing_array[i + 1];
	easting_array[i] = easting_array[i + 1];
      }
      
      std::getline(inFile, str_northing);
      if(inFile.eof()) {
	for(int i = 0; i < num_points_in_line/2; i++) {
	  outFile << start_vel << std::endl;
	}
	break;
      }
      
      std::getline(inFile, str_easting);
      northing_array[num_points_in_line - 1] = strtof(str_northing.c_str(), &pEnd);
      easting_array[num_points_in_line - 1] = strtof(str_easting.c_str(), &pEnd);
    }
  }

private:
  std::ifstream inFile;
  std::ofstream outFile;

  float FindMaxVelocity(float _dist) {
    float dist = _dist;
    float max_vel = 1.5;
    
    if(dist <= 0.5) return max_vel;
    else if(dist <= 1.5) return max_vel*2.0/3.0;
    else return max_vel / 3.0;
  }

};


/***********************************************************************
 *                                                                     *
 *                      MAIN FUNCTION                                  *
 *                                                                     *
 *********************************************************************/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_node");
  
  char inputFile[] = "/home/robot/catkin_ws/src/testing/gps_files/gps_raw.kml";
  char outputFile[] = "/home/robot/catkin_ws/src/testing/gps_files/path.txt";
  char velFile[] = "/home/robot/catkin_ws/src/testing/gps_files/vel_map.txt";
  
  KmlExport *kmlObject = new KmlExport();
  VelMap *velMapObject = new VelMap();
  
  kmlObject->ExportFile(inputFile, outputFile);
  velMapObject->ExportFile(outputFile, velFile);
  
  std::cout << "UTM CONVERSION COMPLETE" << std::endl;
  std::cout << "VELOCITY MAP GENERATED" << std::endl;

  return 0;
}



