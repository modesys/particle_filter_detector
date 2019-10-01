#ifndef USBL_READER_H
#define USBL_READER_H

#include "csv.h"
#include "global_t_stamp.h"
#include <string>

// Setting up ROS messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// Setting up USBL custom messages
#include "ros_float/accuracy.h"
#include "ros_float/ctime.h"
#include "ros_float/depthFromUSBL.h"
#include "ros_float/e.h"
#include "ros_float/h.h"
#include "ros_float/integrity.h"
#include "ros_float/mtime.h"
#include "ros_float/n.h"
#include "ros_float/p.h"
#include "ros_float/prop_time.h"
#include "ros_float/r.h"
#include "ros_float/remote_id.h"
#include "ros_float/rssi.h"
#include "ros_float/u.h"
#include "ros_float/x.h"
#include "ros_float/y.h"
#include "ros_float/z.h"

struct USBL_DATA
{
    int prop_time;
    double accuracy;
    double e;
    unsigned long ctime;
    double h;
    int remote_id;
    int rssi;
    double n;
    double p;
    double depthUSBL;
    double r;
    double u;
    unsigned long mtime;
    double xPosition;
    double yPosition;
    double zPosition;
    int integrity;
    unsigned long timestamp;
};

//struct xyzPosition {
//    double xCov;
//    double yCov;
//    double zCov;
//};

class USBL_reader
{
public:
  USBL_reader(std::string filename);
  bool nextLine();

  USBL_DATA usbl_data;
  ros_float::accuracy accuracyMsg;
  ros_float::ctime ctimeMsg;
  ros_float::depthFromUSBL depthFromUSBLMsg;
  ros_float::e eMsg;
  ros_float::h hMsg;
  ros_float::integrity integraityMsg;
  ros_float::mtime mTimeMsg;
  ros_float::n nMsg;
  ros_float::p pMsg;
  ros_float::prop_time propTimesg;
  ros_float::r rMsg;
  ros_float::remote_id remoteIDMsg;
  ros_float::rssi rssiMsg;
  ros_float::u uMsg;
  ros_float::x xMsg;
  ros_float::y yMsg;
  ros_float::z zMsg;

  // Additional way of adding ROS-non custom messages
  geometry_msgs::Pose poseXYZMsg;
  geometry_msgs::PoseWithCovarianceStamped poseWithCovStamped;
//  geometry_msgs::PoseWithCovariance poseWithCovMsg;


private:
  io::CSVReader<18> usbl_reader;
  unsigned int msgPoseWithCovarianceStamped;
  unsigned int msgNumUSBL;
  //xyzPosition xyzPos;

  void pack_XYZ_Pose_WithCovariance_Msg();
  void pack_Prop_Time_Msg();
  void pack_Accuracy_Msg();
  void pack_E_Msg();
  void pack_CTime_Msg();
  void pack_H_Msg();
  void pack_remote_ID_Msg();
  void pack_Rssi_Msg();
  void pack_N_Msg();
  void pack_P_Msg();
  // void pack_DepthUSBL_Msg();
  void pack_R_Msg();
  void pack_U_Msg();
  void pack_MTime_Msg();
  void pack_XPos_Msg();
  void pack_YPos_Msg();
  void pack_ZPos_Msg();
  void pack_Integrity_Msg();

  // ROS geometry_msgs
  void pack_PoseXYZMsg();

};

#endif // USBL_READER_H
