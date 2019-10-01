#ifndef USBL_RELATIVEREFERENCE_READER_H
#define USBL_RELATIVEREFERENCE_READER_H


#define PI 3.14159265358979323846  /* pi */

#include "csv.h"
#include "global_t_stamp.h"
#include <string>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// Setting up ROS messages
#include "ros_float/accuracy_from_rel_usbl.h"
#include "ros_float/depth_rel_reader.h"
#include "ros_float/remote_id_rel_usbl_reader.h"
#include "ros_float/ship_heading.h"
#include "ros_float/ship_latitude.h"
#include "ros_float/ship_longitude.h"
#include "ros_float/ship_pitch.h"
#include "ros_float/ship_roll.h"
#include "ros_float/target_x.h"
#include "ros_float/target_y.h"
#include "ros_float/target_z.h"


struct USBL_RELATIVE_DATA
{
    unsigned long timestamp;
    double ship_pitch;
    double ship_longitude;
    double ship_latitude;
    double ship_roll;
    int remote_id;
    double depth;
    double longitude;
    double target_z;
    double target_x;
    double target_y;
    double ship_heading;
    double latitude;
    double accuracy;
};

enum surfaceOrUnderwater {
    SURF,
    UND
};

enum status {
    OUT,
    UNDERWATER
};
struct signalsData
{
    status surfaceStatus;
};

struct in_Out
{
    bool surface;
    bool underwater;
};


class USBL_relativeReference_reader
{
public:
  USBL_relativeReference_reader(std::string filename);
  bool nextLine();

  USBL_RELATIVE_DATA usbl_relative_data;
  sensor_msgs::NavSatFix latLongMsg;
  geometry_msgs::PoseWithCovarianceStamped pwcMsg;

  ros_float::accuracy_from_rel_usbl accFromRelUsblMsg;
  ros_float::depth_rel_reader depthRelReader;
  ros_float::remote_id_rel_usbl_reader remIDRelUsblRead;
  ros_float::ship_heading shipHeadMsg;
  ros_float::ship_latitude shipLatMsg;
  ros_float::ship_longitude shipLongMsg;
  ros_float::ship_pitch pitchShipMsg;
  ros_float::ship_roll rollSheepMsg;
  ros_float::target_x targetXMsg;
  ros_float::target_y targetYMsg;
  ros_float::target_z targetZMsg;

private:
  io::CSVReader<14> usbl_ref_reader;

  signalsData sigData;

  unsigned int msgRelLatLon;
  unsigned int msgRelUSBLPose;

  void pack_lat_lon_Msg();

  void pack_accuracyFrom_Rel_Usbl_Msg();
  void pack_depth_rel_reader_Msg();
  void pack_remote_ID_rel_Usbl_Reader_Msg();
  void pack_ship_Heading_Msg();
  void pack_ship_latitude_Msg();
  void pack_ship_longitude_Msg();
  void pack_ship_pitch_Msg();
  void pack_ship_roll_Msg();
  void pack_targetX_Msg();
  void pack_targetY_Msg();
  void pack_targetZ_Msg();
};

#endif // USBL_RELATIVEREFERENCE_READER_H
