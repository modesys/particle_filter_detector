#ifndef COMMAND_READER_H
#define COMMAND_READER_H

#include "csv.h"
#include <string>
#include "global_t_stamp.h"

// Setting up custom messages
#include "ros_float/actuator_mode.h"
#include "ros_float/altitude_check_depth.h"
#include "ros_float/vel_ref.h"
#include "ros_float/abort.h"
#include "ros_float/est_water_depth.h"
#include "ros_float/thrust_offset.h"
#include "ros_float/alt_ref.h"
#include "ros_float/ctl_active.h"
#include "ros_float/volume_ref.h"
#include "ros_float/ctl_modeFromCommand.h"
#include "ros_float/neutral_volume.h"
#include "ros_float/thrust_ref.h"
#include "ros_float/depth_ref.h"
#include "ros_float/fake_altimeter.h"


struct COMMAND_DATA
{
    unsigned long timestamp;
    int actuator_mode;
    int altitude_check_depth;
    int vel_ref;
    int abort;
    int est_water_depth;
    int thrust_offset;
    double alt_ref;
    int ctl_active;
    int volume_ref;
    int ctl_modeFromCommand;
    int neutral_volume;
    //int thrust_ref; Check this because I had to change from int to float
    double thrust_ref;
    int depth_ref;
    int fake_altimeter;
};

class COMMAND_reader
{
public:
  COMMAND_reader(std::string filename);
  bool nextLine();

  COMMAND_DATA command_data;
  ros_float::actuator_mode        actuatorModeMsg;
  ros_float::altitude_check_depth altitudeCheckDepthMsg;
  ros_float::vel_ref              velRefMsg;
  ros_float::abort                abortMsg;
  ros_float::est_water_depth      estWaterDepthMsg;
  ros_float::thrust_offset        thrustOffsetMsg;
  ros_float::alt_ref              altRefMsg;
  ros_float::ctl_active           ctlActiveMsg;
  ros_float::volume_ref           volumeRefMsg;
  ros_float::ctl_modeFromCommand  ctlModeFromCommandMsg;
  ros_float::neutral_volume       neutralVolumeMsg;
  ros_float::thrust_ref           thrustRefMsg;
  ros_float::depth_ref            depthRefMsg;
  ros_float::fake_altimeter       fakeAltimeterMsg;

private:
  io::CSVReader<15> command_reader;
  unsigned int msgNumCommand;

  // Get ready all custom messages
  void pack_Actuator_Mode_Msg();
  void pack_Altitude_Check_Depth_Msg();
  void pack_Vel_Ref_Msg();
  void pack_Abort_Msg();
  void pack_Est_Water_Depth_Msg();
  void pack_Thrust_Offset_Msg();
  void pack_Alt_Ref_Msg();
  void pack_Ctl_Active_Msg();
  void pack_Volume_Ref_Msg();
  void pack_Ctl_ModeFromCommand_Msg();
  void pack_Neutral_Volume_Msg();
  void pack_Thrust_Ref_Msg();
  void pack_Depth_Ref_Msg();
  void pack_Fake_ALtimeter();
};

#endif // COMMAND_READER_H
