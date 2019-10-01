#include "cam_reader.h"

CAM_reader::CAM_reader(std::string filename)
    : cam_reader(filename)
{
    msgNumCam = 0;
    msgOdomNum = 0;
    cam_reader.read_header(io::ignore_extra_column,
                           "imnum", "utctime", "surveytime", "dt", "alt", "stdalt", "distance",
                           "stddistance", "xdistance", "stdxdistance", "ydistance", "stdydistance");
}

bool CAM_reader::nextLine()
{
    if(cam_reader.read_row(cam_data.imNum, cam_data.utctime, cam_data.surveyTime,
                           cam_data.dt, cam_data.alt, cam_data.stdAlt, cam_data.distance,
                           cam_data.stdDistance, cam_data.xDistance, cam_data.stdX_Distance,
                           cam_data.yDistance, cam_data.stdY_Distance))
    {
        // ROS messages
        pack_Cam_Image_Odometry_Msg();
        // custom messages
        pack_Cam_Dt_Msg();
        pack_Cam_Alt_Msg();
        pack_Cam_Dist_Msg();
        pack_Cam_XDist_Msg();
        pack_Cam_YDist_Msg();
        pack_Cam_StdAlt_Msg();
        pack_Cam_StdDist_Msg();
        pack_Cam_Std_X_Dist_Msg();
        pack_Cam_Std_Y_Dist_Msg();
        pack_Cam_SurveyTime_Msg();

        if(odomMsg.header.stamp.sec == 0 && odomMsg.header.stamp.nsec == 0) {
            std::cerr<<"WARNING: CAM message " << msgOdomNum++ << " is valid" << std::endl;
            return nextLine();
        }
        return true;
    }
    else
    {
        return false;
    }

}

// ROS messages
// this messages are in the odom frame and the child
// frame is the camera frame

void CAM_reader::pack_Cam_Image_Odometry_Msg()
{
    timestampToDouble_t currentTime = cam_data.utctime;
    ros::Time stamp(currentTime);
    odomMsg.header.stamp = stamp;
    odomMsg.header.frame_id = "map";
    odomMsg.child_frame_id = "base_link";
    // to prevent from spinning and facing north
    odomMsg.pose.pose.orientation.x = 0.0;
    odomMsg.pose.pose.orientation.y = 0.0;
    odomMsg.pose.pose.orientation.z = 0.0;
    odomMsg.pose.pose.orientation.w = 1.0;
    // set the orientation covariance
    odomMsg.pose.covariance[0]  = 1.0;
    odomMsg.pose.covariance[7]  = 1.0;
    odomMsg.pose.covariance[14] = 1.0;
    odomMsg.pose.covariance[21] = 1.0;
    odomMsg.pose.covariance[28] = 1.0;
    odomMsg.pose.covariance[35] = 1.0;

    odomMsg.header.seq = msgOdomNum;

    // linear velocity
    odomMsg.twist.twist.linear.x = cam_data.xDistance; // deltaX
    odomMsg.twist.twist.linear.y = cam_data.yDistance; // deltaY
    odomMsg.twist.twist.linear.z = 0;
    // covariance for x and y
//    odomMsg.twist.covariance[0] = std::pow(cam_data.stdX_Distance/1000, 2);
//    odomMsg.twist.covariance[7] = std::pow(cam_data.stdY_Distance/1000, 2);
    odomMsg.twist.covariance[0] = 0.05;
    odomMsg.twist.covariance[7] = 0.05;

    odomMsg.twist.covariance[14] = 1.0; // we dont have this data
    odomMsg.twist.covariance[21] = 1.0; // we dont have this data
    odomMsg.twist.covariance[28] = 1.0; // we dont have this data
    odomMsg.twist.covariance[35] = 1.0; // we dont have this data
}

// custom messages
void CAM_reader::pack_Cam_Alt_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    altMsg.header.stamp = stamp;
    altMsg.header.frame_id = "cam_altitude";
    altMsg.header.seq = msgNumCam;
    altMsg.cam_alt_reader = cam_data.alt;
}

void CAM_reader::pack_Cam_Dist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    distMsg.header.stamp = stamp;
    distMsg.header.frame_id = "cam_distance";
    distMsg.header.seq = msgNumCam;
    distMsg.cam_distance_reader = cam_data.distance;
}

void CAM_reader::pack_Cam_Dt_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    dtMsg.header.stamp = stamp;
    dtMsg.header.frame_id = "cam_dt";
    dtMsg.header.seq = msgNumCam;
    dtMsg.cam_dt_reader = cam_data.dt;
}

void CAM_reader::pack_Cam_StdAlt_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    stdAltMsg.header.stamp = stamp;
    stdAltMsg.header.frame_id = "cam_stddevalt";
    stdAltMsg.header.seq = msgNumCam;
    stdAltMsg.cam_std_alt = cam_data.stdAlt;
}

void CAM_reader::pack_Cam_StdDist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    stdDistMsg.header.stamp = stamp;
    stdDistMsg.header.frame_id = "cam_stddist";
    stdDistMsg.header.seq = msgNumCam;
    stdDistMsg.cam_stddistance_reader = cam_data.stdDistance;
}

void CAM_reader::pack_Cam_Std_X_Dist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    stdXMsg.header.stamp = stamp;
    stdXMsg.header.frame_id = "cam_stdx";
    stdXMsg.header.seq = msgNumCam;
    stdXMsg.cam_std_x_dist = cam_data.stdX_Distance;
}

void CAM_reader::pack_Cam_Std_Y_Dist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    stdYMsg.header.stamp = stamp;
    stdYMsg.header.frame_id = "cam_stdy";
    stdYMsg.header.seq = msgNumCam;
    stdYMsg.cam_std_y_dist_reader = cam_data.stdY_Distance;
}

void CAM_reader::pack_Cam_SurveyTime_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    survTimeMsg.header.stamp = stamp;
    survTimeMsg.header.frame_id = "cam_survtime";
    survTimeMsg.header.seq = msgNumCam;
    survTimeMsg.cam_survey_time = cam_data.surveyTime;
}

void CAM_reader::pack_Cam_XDist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    xDistMsg.header.stamp = stamp;
    xDistMsg.header.frame_id = "cam_xdist";
    xDistMsg.header.seq = msgNumCam;
    xDistMsg.cam_x_dist_reader = cam_data.xDistance;
}

void CAM_reader::pack_Cam_YDist_Msg()
{
    timestampToDouble_t currentTime = double(cam_data.utctime);
    ros::Time stamp(currentTime);
    yDistMsg.header.stamp = stamp;
    yDistMsg.header.frame_id = "cam_ydist";
    yDistMsg.header.seq = msgNumCam;
    yDistMsg.cam_y_dist_reader = cam_data.yDistance;
}
