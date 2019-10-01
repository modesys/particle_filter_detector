#ifndef KELLER_READER_H
#define KELLER_READER_H

#include "csv.h"
#include "global_t_stamp.h"
#include <string>
#include <sensor_msgs/FluidPressure.h>
#include "ros_float/pressure.h"
#include "ros_float/depth.h"
#include "ros_float/temperature.h"

struct KELLER_DATA
{
    unsigned long timestamp;
    double depth;
    double temperature;
    double pressure;
};

class Keller_reader
{
public:
  Keller_reader(std::string filename);
  bool nextLine();
  KELLER_DATA keller_data;
  sensor_msgs::FluidPressure depthMessage;
  sensor_msgs::FluidPressure pressurehMessage;
  sensor_msgs::FluidPressure temperatureMessage;

//  ros_float::pressure pMsg;
//  ros_float::temperature tMsg;
//  ros_float::depth dMsg;

private:
  io::CSVReader<4> keller_reader;
  unsigned int msgNumKeller;

  void packDepthMsg();
  void packTemperatureMsg();
  void packPressure();

//  void packPressureMsg();
//  // Custom Pressure Message
//  void packCustom_TemperatureMsg();
//  void packCustom_DepthMsg();
};

#endif // KELLER_READER_H
