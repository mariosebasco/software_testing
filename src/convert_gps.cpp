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
  
  void exportFile(char *inputFile, char *outputFile) {
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
      interpolatePoints(northing, easting);
            
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
  
  kmlObject = new KmlExport();
  
  kmlObject->exportFile(inputFile, outputFile);

  std::cout << "UTM CONVERSION COMPLETE" << std::endl;

  return 0;
}



