#include "alt_reader.h"

ALT_reader::ALT_reader(std::string filename):
    alt_reader(filename)
{
    msgNumAlt = 0;
    alt_reader.read_header(io::ignore_extra_column,
                           "timestamp", "altitude", "good");
}

bool ALT_reader::nextLine()
{
    if(alt_reader.read_row(alt_data.timestamp, alt_data.altitude, alt_data.good))
    {
        pack_Altitude_Msg();
        pack_Good_Msg();

        msgNumAlt++;
        return true;
    }
    else
    {
        return false;
    }
}

void ALT_reader::pack_Altitude_Msg()
{
    timestampToDouble_t currentTime = double(alt_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    altitudeMsg.header.stamp = stamp;
    altitudeMsg.header.frame_id = "alt";
    altitudeMsg.header.seq = msgNumAlt;
    altitudeMsg.altitude = alt_data.altitude;
}

void ALT_reader::pack_Good_Msg()
{
    double doubleTime = double(alt_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    goodMsg.header.stamp = stamp;
    goodMsg.header.frame_id = "good";
    goodMsg.header.seq = msgNumAlt;
    goodMsg.good = alt_data.good;
}
















