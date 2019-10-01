#ifndef STATE_READER_H
#define STATE_READER_H

#include "csv.h"
#include <string>
#include "global_t_stamp.h"
#include "ros_float/altitudeState.h"
#include "ros_float/ctl_force.h"
#include "ros_float/ctl_force_high.h"
#include "ros_float/ctl_force_low.h"
#include "ros_float/ctl_mode.h"
#include "ros_float/ctl_status.h"
#include "ros_float/depthFromState.h"
#include "ros_float/in_water.h"
#include "ros_float/thrust_cmd.h"
#include "ros_float/traj_vel.h"
#include "ros_float/volume_cmd.h"
#include "ros_float/volumeMode_cmd.h"
#include "ros_float/volumeRate_cmd.h"
#include "ros_float/zvelocity.h"

struct STATE_DATA
{
    unsigned long timestamp;
    double state_altitude;
    double ctl_force;
    double ctl_force_high;
    double ctl_force_low;
    int ctl_mode;
    int ctl_status;
    double depth;
    int in_water;
    double thrust_cmd;
    int traj_vel;
    double volume_cmd;
    int volumeMode_cmd;
    int volumeRate_cmd;
    double zvelocity;
};

class STATE_reader
{
public:
  STATE_reader(std::string filename);
  bool nextLine();

  STATE_DATA state_data;
  // Setting up customized messages
  ros_float::altitudeState  altitudeFromStateMsg;
  ros_float::ctl_force      ctlForceMsg;
  ros_float::ctl_force_high ctlForceHightMsg;
  ros_float::ctl_force_low  ctlForceLowMsg;
  ros_float::ctl_mode       ctlModeMsg;
  ros_float::ctl_status     ctlStatusMsg;
  ros_float::depthFromState depthFromStateMsg;
  ros_float::in_water       inWaterMsg;
  ros_float::thrust_cmd     thrustCmdMsg;
  ros_float::traj_vel       trajVelMsg;
  ros_float::volume_cmd     volumeCmdMsg;
  ros_float::volumeMode_cmd volumeModeMsg;
  ros_float::volumeRate_cmd volumeRateMsg;
  ros_float::zvelocity      zVelocityMsg;

private:
    io::CSVReader<15> state_reader;
    unsigned int msgNumState;

    // Get ready all custom messages
    void pack_State_Altitude_Msg();
    void pack_Ctl_ForceMsg();
    void pack_Ctl_Force_High_Msg();
    void pack_Ctl_Force_Low_Msg();
    void pack_Ctl_Mode_Msg();
    void pack_Ctl_Status_Msg();
    void pack_Depth_Msg();
    void pack_In_Water_Msg();
    void pack_Thrust_Cmd_Msg();
    void pack_Traj_Vel_Msg();
    void pack_Volume_Cmd_Msg();
    void pack_Volume_Mode_Msg();
    void pack_Volume_Rate_Msg();
    void pack_ZVelocity_Msg();
};

#endif // STATE_READER_H
