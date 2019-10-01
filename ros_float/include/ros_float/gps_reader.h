#ifndef GPS_READER_H
#define GPS_READER_H

#include "csv.h"
#include <string>
#include "global_t_stamp.h"
#include "ros_float/latitude.h"
#include "ros_float/longitude.h"
#include "ros_float/status.h"
#include "ros_float/altitude.h"

// Comprised for the LogFloat for completeness
#include "ros_float/cmg.h"
#include "ros_float/gps_timestamp.h"
#include "ros_float/mag_var.h"
#include "ros_float/sog.h"

// NavStaFix couild also be used for geo coord
#include <sensor_msgs/NavSatFix.h>

struct GPS_DATA
{
    double latitude;
    double longitude;
    unsigned long timestamp;
    int status;
    double sog;
    double cmg;
    int gps_timestamp;
    int magvar;
};

class GPS_reader
{
public:
  GPS_reader(std::string filename);
  bool nextLine();

  GPS_DATA gps_data;
  ros_float::latitude latMsg;
  ros_float::longitude lonMsg;
  ros_float::status statusMsg;
  ros_float::altitude altMsg;

  ros_float::cmg cmgMsg;
  ros_float::gps_timestamp gps_timeMsg;
  ros_float::mag_var magvarMsg;
  ros_float::sog sogMsg;

  sensor_msgs::NavSatFix fixMsg;
//  sensor_msgs::NavSatFix longitudeMsg;
//  sensor_msgs::NavSatFix altitudeMsg;

private:
  io::CSVReader<8> gps_reader;
  unsigned int msgNumGPS;
  void packFixMsg();

  void packLatMsg();
  void packLonMsg();
  void packStatus();
  void packGPS_Timestamp();
  void packSogMsg();
  void packCmgMsg();
  void packMagVarMsg();
};

#endif // GPS_READER_H
