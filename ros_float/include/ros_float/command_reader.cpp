#include "command_reader.h"

COMMAND_reader::COMMAND_reader(std::string filename):
    command_reader(filename)
{
    msgNumCommand = 0;
    command_reader.read_header(io::ignore_extra_column,
                               "actuator_mode", "altitude_check_depth", "vel_ref", "abort", "est_water_depth",
                               "timestamp", "thrust_offset", "alt_ref", "ctl_active", "volume_ref", "ctl_mode",
                               "neutral_volume", "thrust_ref", "depth_ref", "fake_altimeter");
}

bool COMMAND_reader::nextLine()
{
    if(command_reader.read_row(command_data.actuator_mode, command_data.altitude_check_depth,
                               command_data.vel_ref, command_data.abort, command_data.est_water_depth,
                               command_data.timestamp, command_data.thrust_offset, command_data.alt_ref,
                               command_data.ctl_active, command_data.volume_ref, command_data.ctl_modeFromCommand,
                               command_data.neutral_volume, command_data.thrust_ref, command_data.depth_ref,
                               command_data.fake_altimeter))
    {
        pack_Actuator_Mode_Msg();
        pack_Altitude_Check_Depth_Msg();
        pack_Vel_Ref_Msg();
        pack_Abort_Msg();
        pack_Est_Water_Depth_Msg();
        pack_Thrust_Offset_Msg();
        pack_Alt_Ref_Msg();
        pack_Ctl_Active_Msg();
        pack_Volume_Ref_Msg();
        pack_Ctl_ModeFromCommand_Msg();
        pack_Neutral_Volume_Msg();
        pack_Thrust_Ref_Msg();
        pack_Depth_Ref_Msg();
        pack_Fake_ALtimeter();

        msgNumCommand++;
        return true;
    }
    else
    {
        return false;
    }

}

void COMMAND_reader::pack_Actuator_Mode_Msg()
{
    timestampToDouble_t currentTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    actuatorModeMsg.header.stamp = stamp;
    actuatorModeMsg.header.frame_id = "actuator_mode";
    actuatorModeMsg.header.seq = msgNumCommand;
    actuatorModeMsg.actuator_mode = command_data.actuator_mode;
}

void COMMAND_reader::pack_Altitude_Check_Depth_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    altitudeCheckDepthMsg.header.stamp = stamp;
    altitudeCheckDepthMsg.header.frame_id = "command";
    altitudeCheckDepthMsg.header.seq = msgNumCommand;
    altitudeCheckDepthMsg.altitude_check_depth = command_data.altitude_check_depth;
}

void COMMAND_reader::pack_Vel_Ref_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    velRefMsg.header.stamp = stamp;
    velRefMsg.header.frame_id = "vel_ref";
    velRefMsg.header.seq = msgNumCommand;
    velRefMsg.vel_ref = command_data.vel_ref;
}

void COMMAND_reader::pack_Abort_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    abortMsg.header.stamp = stamp;
    abortMsg.header.frame_id = "abort";
    abortMsg.header.seq = msgNumCommand;
    abortMsg.abort = command_data.abort;
}

void COMMAND_reader::pack_Est_Water_Depth_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    estWaterDepthMsg.header.stamp = stamp;
    estWaterDepthMsg.header.frame_id = "est_water_depth";
    estWaterDepthMsg.header.seq = msgNumCommand;
    estWaterDepthMsg.est_water_depth = command_data.est_water_depth;

}

void COMMAND_reader::pack_Thrust_Offset_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    thrustOffsetMsg.header.stamp = stamp;
    thrustOffsetMsg.header.frame_id = "thrust_offset";
    thrustOffsetMsg.header.seq = msgNumCommand;
    thrustOffsetMsg.thrust_offset = command_data.thrust_offset;
}

void COMMAND_reader::pack_Alt_Ref_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    altRefMsg.header.stamp = stamp;
    altRefMsg.header.frame_id = "alt_ref";
    altRefMsg.header.seq = msgNumCommand;
    altRefMsg.alt_ref = command_data.alt_ref;
}

void COMMAND_reader::pack_Ctl_Active_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlActiveMsg.header.stamp = stamp;
    ctlActiveMsg.header.frame_id = "ctl_active";
    ctlActiveMsg.header.seq = msgNumCommand;
    ctlActiveMsg.ctl_active = command_data.ctl_active;

}

void COMMAND_reader::pack_Volume_Ref_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    volumeRefMsg.header.stamp = stamp;
    volumeRefMsg.header.frame_id = "volume_ref";
    volumeRefMsg.header.seq = msgNumCommand;
    volumeRefMsg.volume_ref = command_data.volume_ref;
}

void COMMAND_reader::pack_Ctl_ModeFromCommand_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    ctlModeFromCommandMsg.header.stamp = stamp;
    ctlModeFromCommandMsg.header.frame_id = "ctl_mode";
    ctlModeFromCommandMsg.header.seq = msgNumCommand;
    ctlModeFromCommandMsg.ctl_modeFromCommand = command_data.ctl_modeFromCommand;
}

void COMMAND_reader::pack_Neutral_Volume_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    neutralVolumeMsg.header.stamp = stamp;
    neutralVolumeMsg.header.frame_id = "neutral_volume";
    neutralVolumeMsg.header.seq = msgNumCommand;
    neutralVolumeMsg.neutral_volume = command_data.neutral_volume;
}

void COMMAND_reader::pack_Thrust_Ref_Msg()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    thrustRefMsg.header.stamp = stamp;
    thrustRefMsg.header.frame_id = "thrust_ref";
    thrustRefMsg.header.seq = msgNumCommand;
    thrustRefMsg.thrust_ref = command_data.thrust_ref;
}

void COMMAND_reader::pack_Depth_Ref_Msg()
{
    timestampToDouble_t currentTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    depthRefMsg.header.stamp = stamp;
    depthRefMsg.header.frame_id = "command";
    depthRefMsg.header.seq = msgNumCommand;
    depthRefMsg.depth_ref = command_data.depth_ref;
}

void COMMAND_reader::pack_Fake_ALtimeter()
{
    double doubleTime = double(command_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    fakeAltimeterMsg.header.stamp = stamp;
    fakeAltimeterMsg.header.frame_id = "fake_altimeter";
    fakeAltimeterMsg.header.seq = msgNumCommand;
    fakeAltimeterMsg.fake_altimeter = command_data.fake_altimeter;
}
