#include "state_reader.h"

STATE_reader::STATE_reader(std::string filename):
    state_reader(filename)
{
    msgNumState = 0;
    state_reader.read_header(io::ignore_extra_column,
                             "timestamp", "altitude", "depth", "volumeMode_cmd", "ctl_force", "ctl_status",
                             "traj_vel", "ctl_force_high", "in_water", "ctl_force_low", "ctl_mode", "thrust_cmd",
                             "zvelocity", "volume_cmd", "volumeRate_cmd");

}

bool STATE_reader::nextLine()
{
    if(state_reader.read_row(state_data.timestamp, state_data.state_altitude, state_data.depth,
                             state_data.volumeMode_cmd, state_data.ctl_force, state_data.ctl_status,
                             state_data.traj_vel, state_data.ctl_force_high, state_data.in_water,
                             state_data.ctl_force_low, state_data.ctl_mode, state_data.thrust_cmd,
                             state_data.zvelocity, state_data.volume_cmd, state_data.volumeRate_cmd))
    {
        pack_State_Altitude_Msg();
        pack_Depth_Msg();
        pack_Volume_Mode_Msg();
        pack_Ctl_ForceMsg();
        pack_Ctl_Status_Msg();
        pack_Traj_Vel_Msg();
        pack_Ctl_Force_High_Msg();
        pack_In_Water_Msg();
        pack_Ctl_Force_Low_Msg();
        pack_Ctl_Mode_Msg();
        pack_Thrust_Cmd_Msg();
        pack_ZVelocity_Msg();
        pack_Volume_Cmd_Msg();
        pack_Volume_Rate_Msg();

        msgNumState++;
        return true;
    }
    else
    {
        return false;
    }
}

void STATE_reader::pack_State_Altitude_Msg()
{
    timestampToDouble_t currentTime = (state_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    altitudeFromStateMsg.header.stamp = stamp;
    altitudeFromStateMsg.header.frame_id = "state";
    altitudeFromStateMsg.header.seq = msgNumState;
    altitudeFromStateMsg.state_altitude = state_data.state_altitude;
}

void STATE_reader::pack_Ctl_ForceMsg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlForceMsg.header.stamp = stamp;
    ctlForceMsg.header.frame_id = "ctl_Force";
    ctlForceMsg.header.seq = msgNumState;
    ctlForceMsg.ctl_force = state_data.ctl_force;
}

void STATE_reader::pack_Ctl_Force_High_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlForceHightMsg.header.stamp = stamp;
    ctlForceHightMsg.header.frame_id = "ctl_force_high";
    ctlForceHightMsg.header.seq = msgNumState;
    ctlForceHightMsg.ctl_force_high = state_data.ctl_force_high;
}

void STATE_reader::pack_Ctl_Force_Low_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlForceLowMsg.header.stamp = stamp;
    ctlForceLowMsg.header.frame_id = "ctl_force_low";
    ctlForceLowMsg.header.seq = msgNumState;
    ctlForceLowMsg.ctl_force_low = state_data.ctl_force_low;
}

void STATE_reader::pack_Ctl_Mode_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlModeMsg.header.stamp = stamp;
    ctlModeMsg.header.frame_id = "ctl_Mode";
    ctlModeMsg.header.seq = msgNumState;
    ctlModeMsg.ctl_mode = state_data.ctl_mode;
}

void STATE_reader::pack_Ctl_Status_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlStatusMsg.header.stamp = stamp;
    ctlStatusMsg.header.frame_id = "ctl_Status";
    ctlStatusMsg.header.seq = msgNumState;
    ctlStatusMsg.ctl_status = state_data.ctl_status;
}

void STATE_reader::pack_Depth_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    depthFromStateMsg.header.stamp = stamp;
    depthFromStateMsg.header.frame_id = "state";
    depthFromStateMsg.header.seq = msgNumState;
    depthFromStateMsg.depth = state_data.depth;
}

void STATE_reader::pack_In_Water_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    inWaterMsg.header.stamp = stamp;
    inWaterMsg.header.frame_id = "in_water";
    inWaterMsg.header.seq = msgNumState;
    inWaterMsg.in_water = state_data.in_water;
}

void STATE_reader::pack_Thrust_Cmd_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    thrustCmdMsg.header.stamp = stamp;
    thrustCmdMsg.header.frame_id = "thrust_cmd";
    thrustCmdMsg.header.seq = msgNumState;
    thrustCmdMsg.thrust_cmd = state_data.thrust_cmd;
}

void STATE_reader::pack_Traj_Vel_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    trajVelMsg.header.stamp = stamp;
    trajVelMsg.header.frame_id = "traj_vel";
    trajVelMsg.header.seq = msgNumState;
    trajVelMsg.traj_vel = state_data.traj_vel;
}

void STATE_reader::pack_Volume_Cmd_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    volumeCmdMsg.header.stamp = stamp;
    volumeCmdMsg.header.frame_id = "volume_cmd";
    volumeCmdMsg.header.seq = msgNumState;
    volumeCmdMsg.volume_cmd = state_data.volume_cmd;
}

void STATE_reader::pack_Volume_Mode_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    volumeModeMsg.header.stamp = stamp;
    volumeModeMsg.header.frame_id = "volumeMode_cmd";
    volumeModeMsg.header.seq = msgNumState;
    volumeModeMsg.volumeMode_cmd = state_data.volumeMode_cmd;
}

void STATE_reader::pack_Volume_Rate_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    volumeRateMsg.header.stamp = stamp;
    volumeRateMsg.header.frame_id = "volumeRate_cmd";
    volumeRateMsg.header.seq = msgNumState;
    volumeRateMsg.volumeRate_cmd = state_data.volumeRate_cmd;
}

void STATE_reader::pack_ZVelocity_Msg()
{
    double doubleTime = double(state_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    zVelocityMsg.header.stamp = stamp;
    zVelocityMsg.header.frame_id = "zvelocity";
    zVelocityMsg.header.seq = msgNumState;
    zVelocityMsg.zvelocity = state_data.zvelocity;
}
