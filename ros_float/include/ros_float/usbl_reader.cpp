#include "usbl_reader.h"

USBL_reader::USBL_reader(std::string filename):
    usbl_reader(filename)
{
    msgPoseWithCovarianceStamped = 0;
    usbl_reader.read_header(io::ignore_extra_column,
                            "prop_time", "accuracy", "e", "ctime", "h", "remote_id",
                            "rssi", "n", "p", "depth", "r", "u", "mtime",
                            "y", "x", "z", "integrity", "timestamp");
}

bool USBL_reader::nextLine()
{
    if(usbl_reader.read_row(usbl_data.prop_time, usbl_data.accuracy, usbl_data.e,
                            usbl_data.ctime, usbl_data.h, usbl_data.remote_id,
                            usbl_data.rssi, usbl_data.n, usbl_data.p,
                            usbl_data.depthUSBL, usbl_data.r, usbl_data.u,
                            usbl_data.mtime, usbl_data.yPosition, usbl_data.xPosition,
                            usbl_data.zPosition, usbl_data.integrity , usbl_data.timestamp))
    {
        pack_Prop_Time_Msg();
        pack_Accuracy_Msg();
        pack_E_Msg();
        pack_CTime_Msg();
        pack_H_Msg();
        pack_remote_ID_Msg();
        pack_Rssi_Msg();
        pack_N_Msg();
        pack_P_Msg();
        pack_R_Msg();
        pack_U_Msg();
        pack_MTime_Msg();
        pack_Integrity_Msg();
        pack_XYZ_Pose_WithCovariance_Msg();

        msgPoseWithCovarianceStamped++;

        if(poseWithCovStamped.header.stamp.sec==0 && poseWithCovStamped.header.stamp.nsec==0){
            std::cerr<<"WARNING: USBL message " << msgPoseWithCovarianceStamped++ << " is invalid" << std::endl;
            return(nextLine());
        }
        return true;
    }
    else
    {
        return false;
    }
}

void USBL_reader::pack_XYZ_Pose_WithCovariance_Msg()
{
    timestampToDouble_t currentTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    poseWithCovStamped.header.stamp = stamp;
    poseWithCovStamped.header.frame_id = "ship";
    poseWithCovStamped.header.seq = msgPoseWithCovarianceStamped;
    poseWithCovStamped.pose.pose.position.x = usbl_data.xPosition;
    poseWithCovStamped.pose.pose.position.y = usbl_data.yPosition;
    poseWithCovStamped.pose.pose.position.z = usbl_data.zPosition;
}

void USBL_reader::pack_Prop_Time_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    propTimesg.header.stamp = stamp;
    propTimesg.header.frame_id = "prop_time";
    propTimesg.header.seq = msgNumUSBL;
    propTimesg.prop_time = usbl_data.prop_time;
}

void USBL_reader::pack_Accuracy_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    accuracyMsg.header.stamp = stamp;
    accuracyMsg.header.frame_id = "accuracy";
    accuracyMsg.header.seq = msgNumUSBL;
    accuracyMsg.accuracy = usbl_data.accuracy;
}

void USBL_reader::pack_E_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    eMsg.header.stamp = stamp;
    eMsg.header.frame_id = "e";
    eMsg.header.seq = msgNumUSBL;
    eMsg.e_reader = usbl_data.e;
}

void USBL_reader::pack_CTime_Msg()
{
    ctime_t currentCTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(currentCTime);
    ctimeMsg.header.stamp = stamp;
    ctimeMsg.header.frame_id = "ctime";
    ctimeMsg.header.seq = msgNumUSBL;
    ctimeMsg.ctime = usbl_data.ctime;
}

void USBL_reader::pack_H_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    hMsg.header.stamp = stamp;
    hMsg.header.frame_id = "h";
    hMsg.header.seq = msgNumUSBL;
    hMsg.h_reader = usbl_data.h;
}

void USBL_reader::pack_remote_ID_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    remoteIDMsg.header.stamp = stamp;
    remoteIDMsg.header.frame_id = "remote_id_reader";
    remoteIDMsg.header.seq = msgNumUSBL;
    remoteIDMsg.remote_id_reader = usbl_data.remote_id;
}

void USBL_reader::pack_Rssi_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    rssiMsg.header.stamp = stamp;
    rssiMsg.header.frame_id = "rssi";
    rssiMsg.header.seq = msgNumUSBL;
    rssiMsg.rssi = usbl_data.rssi;
}

void USBL_reader::pack_N_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    nMsg.header.stamp = stamp;
    nMsg.header.frame_id = "n";
    nMsg.header.seq = msgNumUSBL;
    nMsg.n_reader = usbl_data.n;
}

void USBL_reader::pack_P_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    pMsg.header.stamp = stamp;
    pMsg.header.frame_id = "p";
    pMsg.header.seq = msgNumUSBL;
    pMsg.p_reader = usbl_data.p;
}

//void USBL_reader::pack_DepthUSBL_Msg()
//{
//    double doubleTime = double(usbl_data.timestamp)/1e6;
//    ros::Time stamp(doubleTime);
//    depthFromUSBLMsg.header.stamp = stamp;
//    depthFromUSBLMsg.header.frame_id = "/depth_from_USBL";
//    depthFromUSBLMsg.header.seq = msgNumUSBL;
//    depthFromUSBLMsg.depthFromUSBL = usbl_data.depthUSBL;
//}

void USBL_reader::pack_R_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    rMsg.header.stamp = stamp;
    rMsg.header.frame_id = "r";
    rMsg.header.seq = msgNumUSBL;
    rMsg.r_reader = usbl_data.r;
}

void USBL_reader::pack_U_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    uMsg.header.stamp = stamp;
    uMsg.header.frame_id = "u";
    uMsg.header.seq = msgNumUSBL;
    uMsg.u_reader = usbl_data.u;
}

void USBL_reader::pack_MTime_Msg()
{
    mtime_t currentMTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(currentMTime);
    mTimeMsg.header.stamp = stamp;
    mTimeMsg.header.frame_id = "mtime";
    mTimeMsg.header.seq = msgNumUSBL;
    mTimeMsg.mtime = usbl_data.mtime;
}

void USBL_reader::pack_XPos_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    xMsg.header.stamp = stamp;
    xMsg.header.frame_id = "x_pos";
    xMsg.header.seq = msgNumUSBL;
    xMsg.x_pos = usbl_data.xPosition;
}

void USBL_reader::pack_YPos_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    yMsg.header.stamp = stamp;
    yMsg.header.frame_id = "y_pos";
    yMsg.header.seq = msgNumUSBL;
    yMsg.y_pos = usbl_data.yPosition;
}

void USBL_reader::pack_ZPos_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    zMsg.header.stamp = stamp;
    zMsg.header.frame_id = "z_pos";
    zMsg.header.seq = msgNumUSBL;
    zMsg.z_pos = usbl_data.zPosition;
}

void USBL_reader::pack_Integrity_Msg()
{
    double doubleTime = double(usbl_data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    integraityMsg.header.stamp = stamp;
    integraityMsg.header.frame_id = "integrity";
    integraityMsg.header.seq = msgNumUSBL;
    integraityMsg.intergrity = usbl_data.integrity;
}

