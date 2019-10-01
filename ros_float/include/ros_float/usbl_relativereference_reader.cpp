#include "usbl_relativereference_reader.h"

USBL_relativeReference_reader::USBL_relativeReference_reader(std::string filename):
    usbl_ref_reader(filename)
{
    msgRelUSBLPose = 0;
    msgRelLatLon = 0;
    usbl_ref_reader.read_header(io::ignore_extra_column,
                                "utime", "ship_pitch", "ship_longitude",
                                "ship_latitude", "ship_roll", "remote_id",
                                "depth", "longitude", "target_z", "target_x",
                                "target_y", "ship_heading", "latitude", "accuracy");
}

bool USBL_relativeReference_reader::nextLine()
{
    if(usbl_ref_reader.read_row(usbl_relative_data.timestamp, usbl_relative_data.ship_pitch,
                                usbl_relative_data.ship_longitude, usbl_relative_data.ship_latitude,
                                usbl_relative_data.ship_roll, usbl_relative_data.remote_id, usbl_relative_data.depth,
                                usbl_relative_data.longitude, usbl_relative_data.target_x,
                                usbl_relative_data.target_y, usbl_relative_data.target_z,
                                usbl_relative_data.ship_heading, usbl_relative_data.latitude, usbl_relative_data.accuracy))
    {
        pack_lat_lon_Msg();
        pack_ship_Heading_Msg();
        pack_ship_latitude_Msg();
        pack_ship_longitude_Msg();
        pack_ship_pitch_Msg();
        pack_ship_roll_Msg();
        pack_targetX_Msg();
        pack_targetY_Msg();
        pack_targetZ_Msg();
        pack_accuracyFrom_Rel_Usbl_Msg();
        pack_depth_rel_reader_Msg();
        pack_remote_ID_rel_Usbl_Reader_Msg();

        if(latLongMsg.header.stamp.sec==0 && latLongMsg.header.stamp.nsec==0){
            std::cerr<<"WARNING: LAT_LON message " << msgRelLatLon++ << " is invalid" << std::endl;
            return(nextLine());
        } else if (pwcMsg.header.stamp.sec==0 && pwcMsg.header.stamp.nsec==0){
            std::cerr<<"WARNING: Pose message" << msgRelUSBLPose++ << " is invalid" << std::endl;
        }
        return true;
    }
    else
    {
        return false;
    }
}

void USBL_relativeReference_reader::pack_lat_lon_Msg()
{
    timestampToDouble_t currentTime = double(usbl_relative_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    latLongMsg.header.stamp = stamp;
    latLongMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

//    if(sigData.surfaceStatus == OUT) { // on the surface
//        latLongMsg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
//    }
//    else // underwater
//    {
//        latLongMsg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
//    }

    // calculate the posewith covstamped

    latLongMsg.header.frame_id = "usbl";
    latLongMsg.header.seq = msgRelLatLon;
    latLongMsg.latitude = ((usbl_relative_data.latitude)*180)/PI;
    latLongMsg.longitude = ((usbl_relative_data.longitude)*180)/PI;

    // Pose with covariance
    pwcMsg.header.stamp = stamp;
    pwcMsg.header.frame_id = "utm";
    pwcMsg.header.seq = msgRelUSBLPose;
    pwcMsg.pose.pose.position.x = usbl_relative_data.longitude;
    pwcMsg.pose.pose.position.y = usbl_relative_data.latitude;
}

void USBL_relativeReference_reader::pack_ship_Heading_Msg()
{

}

void USBL_relativeReference_reader::pack_ship_latitude_Msg()
{

}

void USBL_relativeReference_reader::pack_ship_longitude_Msg()
{

}

void USBL_relativeReference_reader::pack_ship_pitch_Msg()
{

}

void USBL_relativeReference_reader::pack_ship_roll_Msg()
{

}

void USBL_relativeReference_reader::pack_targetX_Msg()
{

}

void USBL_relativeReference_reader::pack_targetY_Msg()
{

}

void USBL_relativeReference_reader::pack_targetZ_Msg()
{

}

void USBL_relativeReference_reader::pack_accuracyFrom_Rel_Usbl_Msg()
{

}

void USBL_relativeReference_reader::pack_depth_rel_reader_Msg()
{

}

void USBL_relativeReference_reader::pack_remote_ID_rel_Usbl_Reader_Msg()
{

}
