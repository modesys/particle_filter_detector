#include <ros/ros.h>

#include <rosbag/bag.h>
#include <memory.h>

#include "../include/ros_float/um6_reader.h"
#include "../include/ros_float/keller_reader.h"
#include "../include/ros_float/gps_reader.h"
#include "../include/ros_float/alt_reader.h"
#include "../include/ros_float/state_reader.h"
#include "../include/ros_float/command_reader.h"
#include "../include/ros_float/usbl_reader.h"
#include "../include/ros_float/usbl_relativereference_reader.h"
#include "../include/ros_float/cam_reader.h"

int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;
    rosbag::Bag bag;
    bag.open("test.bag", rosbag::bagmode::Write);
    std::string fstring = "/home/emanuele/Desktop/um6_data_t_UM6_DATA.csv";
    UM6_reader um6(fstring);
    while(um6.nextLine()) {
        bag.write("imu/data", um6.imuMsg.header.stamp , um6.imuMsg);
        bag.write("imu/mag", um6.magMsg.header.stamp, um6.magMsg);
    }
    std::string fstringKel = "/home/emanuele/Desktop/kellerpressure_data_t_KELLER_DATA.csv";
    Keller_reader Kel(fstringKel);
    int counterKel =0;
    while(Kel.nextLine()) {
        counterKel++;
        bag.write("fluid_pressure/pressure", Kel.pressurehMessage.header.stamp, Kel.pressurehMessage);
        bag.write("fluid_pressure/temp", Kel.temperatureMessage.header.stamp, Kel.temperatureMessage);
        bag.write("fluid_pressure/depth", Kel.depthMessage.header.stamp, Kel.depthMessage);
    }
    std::string fstringGPS = "/home/emanuele/Desktop/gps_gprmc_t_GPS_GPRMC_DATA.csv";
    GPS_reader GPS(fstringGPS);
    int counterGPS = 0;
    while(GPS.nextLine()) {
        counterGPS++;
        bag.write("gps/data", GPS.fixMsg.header.stamp, GPS.fixMsg);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringSTATE = "/home/emanuele/Desktop/float_controller_state_t_CTL_STATE.csv";
    STATE_reader STATE(fstringSTATE);
    int counterSTATE = 0;;
    while(STATE.nextLine()) {
        counterSTATE++;
        bag.write("state/altitude_from_state", STATE.depthFromStateMsg.header.stamp, STATE.altitudeFromStateMsg);
        bag.write("state/depth_from_state", STATE.depthFromStateMsg.header.stamp, STATE.depthFromStateMsg);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringALT = "/home/emanuele/Desktop/floatalt_data_t_ALT_DATA.csv";
    ALT_reader ALT(fstringALT);
    int counterALT = 0;
    while (ALT.nextLine()) {
        counterALT++;
        bag.write("alt/altitude", ALT.altitudeMsg.header.stamp, ALT.altitudeMsg);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringCOMMAND = "/home/emanuele/Desktop/float_controller_cmd_t_CTL_COMMAND.csv";
    COMMAND_reader COMMAND(fstringCOMMAND);
    int counterCOMMAND = 0;
    while(COMMAND.nextLine()) {
        counterCOMMAND++;
        bag.write("command/depth_ref", COMMAND.depthRefMsg.header.stamp, COMMAND.depthRefMsg);
        bag.write("command/altitude_check_depth", COMMAND.altitudeCheckDepthMsg.header.stamp, COMMAND.altitudeCheckDepthMsg);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringUSBL = "/home/emanuele/Desktop/evologics_usbl_t_FALKOR_EVO_USBLFIX_PFLOAT.csv";
    USBL_reader USBL(fstringUSBL);
    int counterUSBL = 0;
    while(USBL.nextLine()) {
        counterUSBL++;
        bag.write("ship/pose", USBL.poseWithCovStamped.header.stamp, USBL.poseWithCovStamped);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringRelativeRefUSBL = "/home/emanuele/Desktop/20180206_usbl_fix_t.csv";
    USBL_relativeReference_reader USBLRELREF(fstringRelativeRefUSBL);
    int counterUSBLREF = 0;
    while(USBLRELREF.nextLine()) {
        counterUSBLREF++;
        bag.write("usbl/latitude_longitude", USBLRELREF.latLongMsg.header.stamp, USBLRELREF.latLongMsg);
        bag.write("usbl/pose_with_covariance", USBLRELREF.pwcMsg.header.stamp, USBLRELREF.pwcMsg);
        // If additional messages are needed add them here: Templates are ready
    }
    std::string fstringCam = "/home/emanuele/Desktop/20180206_1733_rot_filt.csv";
    CAM_reader CAM(fstringCam);
    int counterCAM = 0;
    while(CAM.nextLine()) {
        counterCAM++;
        bag.write("cam/odometry", CAM.odomMsg.header.stamp, CAM.odomMsg);
        // If additional messages are needed add them here: Templates are ready
    }
}

