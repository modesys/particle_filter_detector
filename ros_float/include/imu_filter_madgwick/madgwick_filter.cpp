#include "madgwick_filter.h"

namespace madgwick
{
    Madgwick::Madgwick()
    {

    }

    MADGWICK_reader::MADGWICK_reader(std::string filename, ros::NodeHandle _n):
        n(_n),madg_imu_reader(filename)
    {
        imuMadgNum = 0;
        magMadgNum = 0;
        madg_imu_reader.read_header(io::ignore_extra_column,
                                   "magz","magy","magx",
                                   "accelz","accelx","accely","timestamp",
                                   "phi","psi","theta",
                                   "gyrox","gyroy","gyroz");

        // subscriber and publisher inside the class.
        // in particular inside constructor
        pub1 = n.advertise<geometry_msgs::QuaternionStamped>("quaternion", 1);
        sub = n.subscribe("imu0", 10, &MADGWICK_reader::filter_function, this);

    }

    bool MADGWICK_reader::nextLine()
    {
        if(madg_imu_reader.read_row(madgwick_data.magz, madgwick_data.magy, madgwick_data.magx,     // MagneticX, MAgneticY, MagneticZ
                           madgwick_data.accelz, madgwick_data.accelx, madgwick_data.accely, madgwick_data.timestamp, // AccX, AccY, AccZ
                           madgwick_data.phi, madgwick_data.psi, madgwick_data.theta,      // Angles orientations
                           madgwick_data.gyrox, madgwick_data.gyroy,madgwick_data.gyroz))  // Angulat Velocity X, Y, Z
        {
            pack_Imu_Madgwick_Msg();
            pack_Mag_Madgwick_Msg();

            imuMadgNum++;
            magMadgNum++;

            if(imuMadgwickMsg.header.stamp.sec==0 && imuMadgwickMsg.header.stamp.nsec==0)
            {
                std::cerr<<"WARNING: IMU message " << imuMadgNum++ << " is invalid" << std::endl;
                return(nextLine());
            } else if(magMedgwickMsg.header.stamp.sec==0&& magMedgwickMsg.header.stamp.nsec==0)
            {
                std::cerr<<"WARNINIG: MAG message " << magMadgNum++ << " is invlid" << std::endl;
                return(nextLine());
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    // CASE 1: writing a function that takes -->  (const sensor_msgs::Imu &msg)
    void MADGWICK_reader::filter_function(const sensor_msgs::Imu &msg)
    {
        timestampToDouble_t currentTime = (madgwick_data.timestamp)/1e6;
        ros::Time stamp(currentTime);
        header = msg.header;
        //  q.linear_acceleration = msg.linear_acceleration;
        //  q.angular_velocity = msg.angular_velocity;
        ax = float(msg.linear_acceleration.x);
        ay = float(msg.linear_acceleration.y);
        az = float(msg.linear_acceleration.z);

        gx = float(msg.angular_velocity.x);
        gy = float(msg.angular_velocity.y);
        gz = float(msg.angular_velocity.z);

        ROS_INFO("gx=%f, gy=%f, gz=%f ", gx, gy, gz);
        ROS_INFO("beta=%f", beta);

        madgwickIMU(gx, gy, gz, ax, ay, az);

        q.header = header;
        q.quaternion.w = q0;
        q.quaternion.x = q1;
        q.quaternion.y = q2;
        q.quaternion.z = q3;

         ROS_INFO(" q0=%f, q1=%f, q2=%f, q3=%f ", q0, q1, q2, q3);
         qua2Euler(q);

    }


    // CASE 2: packing messages both for Imu and Mag

    void MADGWICK_reader::pack_Imu_Madgwick_Msg()
    {
        timestampToDouble_t currentTime = (madgwick_data.timestamp)/1e6;
        ros::Time stamp(currentTime);
        imuMadgwickMsg.header.stamp = stamp;
        imuMadgwickMsg.header.frame_id = "/imu_madgwick";
        imuMadgwickMsg.header.seq = imuMadgNum;

        tf2::Quaternion q;
        q.setRPY( madgwick_data.phi, madgwick_data.psi, madgwick_data.theta );

        imuMadgwickMsg.orientation.x = q.x();
        imuMadgwickMsg.orientation.y = q.y();
        imuMadgwickMsg.orientation.z = q.z();

        imuMadgwickMsg.angular_velocity.x = madgwick_data.gyrox;
        imuMadgwickMsg.angular_velocity.y = madgwick_data.gyroy;
        imuMadgwickMsg.angular_velocity.z = madgwick_data.gyroz;

    }

    void MADGWICK_reader::pack_Mag_Madgwick_Msg()
    {
        timestampToDouble_t currentTime = (madgwick_data.timestamp)/1e6;
        ros::Time stamp(currentTime);
        magMedgwickMsg.header.stamp = stamp;
        magMedgwickMsg.header.frame_id = "/mag_madgwick";
        magMedgwickMsg.header.seq = magMadgNum;
        magMedgwickMsg.magnetic_field.x = madgwick_data.magx;
        magMedgwickMsg.magnetic_field.y = madgwick_data.magy;
        magMedgwickMsg.magnetic_field.z = madgwick_data.magz;

    }

}
