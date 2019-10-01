#ifndef MADGWICK_H
#define MADGWICK_H

#include "csv.h"

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <math.h>
#include <string>
#include <cstdint>
#include <stdint.h>
#include <cstring>


namespace madgwick
{
    using timestamp_t = uint64_t;
    using timestampToDouble_t = double;

    struct IMU_MADGWICK_DATA
    {
        double magz;
        double magy;
        double magx;
        float accelz;
        float accelx;
        float accely;
        unsigned long timestamp; /// FIXME uint64_t?
        double phi;   // orientation
        double psi;   // orientation
        double theta; // orientation
        double gyrox; // angular velocity x
        double gyroy; // angular velocity y
        double gyroz; // angular velocity z
    };


    class Madgwick
    {
    public:
      Madgwick();

    };

    class MADGWICK_reader
    {

        // subscriber and publisher inside the class
        ros::Subscriber sub;
        ros::Publisher pub1;
        ros::NodeHandle n;

    public:
        MADGWICK_reader(std::string filename, ros::NodeHandle _n);
        bool nextLine();
        bool nextLineWithSupport();

        IMU_MADGWICK_DATA madgwick_data;

        sensor_msgs::Imu imuMadgwickMsg;
        sensor_msgs::MagneticField magMedgwickMsg;

        geometry_msgs::QuaternionStamped q;
        geometry_msgs::Vector3Stamped v;
        geometry_msgs::TransformStamped q_trans;
        float sampleFreq;
        double beta;
        double q0=1.0, q1=0.0, q2=0.0, q3=0.0;
        std_msgs::Header header;
        float ax, ay, az, gx, gy, gz;
        ros::Duration dtime;
        float dt;

        float invSqrt();
        void qua2Euler(geometry_msgs::QuaternionStamped);
        void madgwickIMU(float gx, float gy, float gz, float ax, float ay, float az);
        void filter_function(const sensor_msgs::Imu &msg);

    private:
        io::CSVReader<13> madg_imu_reader;
        unsigned int imuMadgNum;
        unsigned int magMadgNum;
        void packMadgMagMsg();
        void pack_Imu_Madgwick_Msg();
        void pack_Mag_Madgwick_Msg();
};
}

#endif // MADGWICK_H
