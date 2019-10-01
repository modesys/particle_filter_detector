#ifndef UM6_READER_H
#define UM6_READER_H

#include "csv.h"
#include <memory>
#include <string>
#include "global_t_stamp.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

struct UM6_DATA{
    double magz;
    double magy;
    double magx;
    double accelz;
    double accelx;
    double accely;
    unsigned long timestamp; /// FIXME uint64_t?
    double phi;   // orientation
    double psi;   // orientation
    double theta; // orientation
    double gyrox; // angular velocity x
    double gyroy; // angular velocity y
    double gyroz; // angular velocity z
};


class UM6_reader
{
public:
    UM6_reader(std::string filename);
    bool nextLine();

    UM6_DATA data;
    sensor_msgs::Imu imuMsg;
    sensor_msgs::MagneticField magMsg;
    sensor_msgs::Imu angVelMsg;
    sensor_msgs::Imu orientationMsg;
    geometry_msgs::TwistWithCovarianceStamped twcMsg;
    nav_msgs::Odometry odomMsg;

private:
    void packImuMsg();
    void packTwistMsg();
    void packOdometryMsg();

    io::CSVReader<13> reader;
    unsigned int imuNum;
    unsigned int magNum;
    unsigned int msgNum;
    unsigned int msgTwist;


    void packMagMsg();
    void packAngVelMsg();
    void packOrientationMsg();

    double roll, pitch, yaw;

};

#endif // UM6_READER_H
