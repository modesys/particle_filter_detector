#include "gps_reader.h"

GPS_reader::GPS_reader(std::string filename):
    gps_reader(filename)
{
    msgNumGPS = 0;
    gps_reader.read_header(io::ignore_extra_column,
                           "lat", "lon", "timestamp",
                           "status", "sog", "cmg",
                           "gps_timestamp", "mag_var");
}

bool GPS_reader::nextLine()
{
    if(gps_reader.read_row(gps_data.latitude, gps_data.longitude,
                           gps_data.timestamp, gps_data.status,
                           gps_data.sog, gps_data.cmg,
                           gps_data.gps_timestamp, gps_data.magvar))
    {
        // Messages required
        packFixMsg();

        // Custom messages if needed: here for completeness
        packLatMsg();
        packLonMsg();
        packStatus();
        packSogMsg();
        packCmgMsg();
        packGPS_Timestamp();
        packMagVarMsg();

        msgNumGPS++;

        if(fixMsg.header.stamp.sec==0 && fixMsg.header.stamp.nsec==0){
            std::cerr<<"WARNING: GPS message " << msgNumGPS++ << " is invalid" << std::endl;
            return(nextLine());
        }
        return true;
    }
    else
    {
        return false;
    }
}

///
/// \brief Packs up fix message from input file
///
void GPS_reader::packFixMsg()
{
    timestampToDouble_t currentTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    fixMsg.header.stamp = stamp;
    /// \todo ask croman for confirmation
    if(gps_data.status == 1)
    { // If the float is on the surface
        fixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    }
    else // If the float is underwater
    {
        fixMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    }
    fixMsg.header.frame_id = "gps";
    fixMsg.header.seq = msgNumGPS;
    fixMsg.latitude = gps_data.latitude;
    fixMsg.longitude = gps_data.longitude;
    fixMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

void GPS_reader::packLatMsg()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    latMsg.header.stamp = stamp;
    latMsg.header.frame_id = "gps";
    latMsg.header.seq = msgNumGPS;
    latMsg.latitude = gps_data.latitude;
}

void GPS_reader::packLonMsg()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    lonMsg.header.stamp = stamp;
    lonMsg.header.frame_id = "gps";
    lonMsg.header.seq = msgNumGPS;
    lonMsg.longitude = gps_data.longitude;
}

void GPS_reader::packStatus()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    statusMsg.header.stamp = stamp;
    statusMsg.header.frame_id = "gps";
    statusMsg.header.seq = msgNumGPS;
    statusMsg.status = gps_data.status;
}

void GPS_reader::packGPS_Timestamp()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    gps_timeMsg.header.stamp = stamp;
    gps_timeMsg.header.frame_id = "gps";
    gps_timeMsg.header.seq = msgNumGPS;
    gps_timeMsg.gps_timestamp = gps_data.gps_timestamp;
}

void GPS_reader::packSogMsg()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    sogMsg.header.stamp = stamp;
    sogMsg.header.frame_id = "gps";
    sogMsg.header.seq = msgNumGPS;
    sogMsg.sog = gps_data.sog;
}

void GPS_reader::packCmgMsg()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    cmgMsg.header.stamp = stamp;
    cmgMsg.header.frame_id = "gps";
    cmgMsg.header.seq = msgNumGPS;
    cmgMsg.cmg = gps_data.cmg;
}

void GPS_reader::packMagVarMsg()
{
    double doubleTime = double(gps_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    magvarMsg.header.stamp = stamp;
    magvarMsg.header.frame_id = "gps";
    magvarMsg.header.seq = msgNumGPS;
    magvarMsg.mag_var = gps_data.magvar;
}










