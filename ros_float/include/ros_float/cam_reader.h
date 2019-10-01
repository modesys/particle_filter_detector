#ifndef CAM_READER_H
#define CAM_READER_H

#include <string>

#include "csv.h"
#include "global_t_stamp.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

// including custom messages
#include "ros_float/alt.h"
#include "ros_float/distance.h"
#include "ros_float/dt.h"
#include "ros_float/imnum.h"
#include "ros_float/stdalt.h"
#include "ros_float/stddistance.h"
#include "ros_float/std_X_distance.h"
#include "ros_float/std_Y_distance.h"
#include "ros_float/surveytime.h"
#include "ros_float/xdistance.h"
#include "ros_float/ydistance.h"

struct CAM_DATA
{
    double imNum;
    double utctime;
    double surveyTime;
    double dt;
    double alt;
    double stdAlt;
    double distance;
    double stdDistance;
    double xDistance;
    double stdX_Distance;
    double yDistance;
    double stdY_Distance;
};

class CAM_reader
{
public:
  CAM_reader(std::string filename);
  bool nextLine();
  CAM_DATA cam_data;

  // Setting up ROS messages for cam images
  nav_msgs::Odometry        odomMsg;

  // Setting up custom messages
  ros_float::alt            altMsg;
  ros_float::distance       distMsg;
  ros_float::dt             dtMsg;
  ros_float::stdalt         stdAltMsg;
  ros_float::stddistance    stdDistMsg;
  ros_float::std_X_distance stdXMsg;
  ros_float::std_Y_distance stdYMsg;
  ros_float::surveytime     survTimeMsg;
  ros_float::xdistance      xDistMsg;
  ros_float::ydistance      yDistMsg;

private:
  io::CSVReader<12> cam_reader;
  unsigned int msgNumCam;
  unsigned int msgOdomNum;

  // packing ROS standard messages
  void pack_Cam_Image_Odometry_Msg();

  // packing custom messages
  void pack_Cam_Alt_Msg();
  void pack_Cam_Dist_Msg();
  void pack_Cam_Dt_Msg();
  void pack_Cam_StdAlt_Msg();
  void pack_Cam_StdDist_Msg();
  void pack_Cam_Std_X_Dist_Msg();
  void pack_Cam_Std_Y_Dist_Msg();
  void pack_Cam_SurveyTime_Msg();
  void pack_Cam_XDist_Msg();
  void pack_Cam_YDist_Msg();


};

#endif // CAM_READER_H
