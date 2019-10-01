#ifndef ALT_READER_H
#define ALT_READER_H

#include "csv.h"
#include <string>
#include "global_t_stamp.h"
#include "ros_float/altitude.h"
#include "ros_float/good.h"

struct ALT_DATA
{
    unsigned long timestamp;
    double altitude;
    int good;
};

class ALT_reader
{
public:
  ALT_reader(std::string filename);
  bool nextLine();

  ALT_DATA alt_data;
  ros_float::altitude altitudeMsg;
  ros_float::good goodMsg;

private:
  io::CSVReader<3> alt_reader;
  unsigned int msgNumAlt;
  void pack_Altitude_Msg();
  void pack_Good_Msg();
};

#endif // ALT_READER_H
