#include "um6_reader.h"

UM6_reader::UM6_reader(std::string filename):
    reader(filename)
{
    imuNum =   0;
    magNum =   0;
    msgNum =   0;
    msgTwist = 0;

    reader.read_header(io::ignore_extra_column,
                       "magz","magy","magx",
                       "accelz","accelx","accely","timestamp",
                       "phi","psi","theta",
                       "gyrox","gyroy","gyroz");
}

bool UM6_reader::nextLine(){
    if(reader.read_row(data.magz, data.magy, data.magx,     // MagneticX, MAgneticY, MagneticZ
                       data.accelz, data.accelx, data.accely, data.timestamp, // AccX, AccY, AccZ
                       data.phi, data.psi, data.theta,      // Angles orientations
                       data.gyrox, data.gyroy,data.gyroz))  // Angulat Velocity X, Y, Z
    {
        packImuMsg();
        packTwistMsg();
        packOdometryMsg();
        packMagMsg();
        packAngVelMsg();
        packOrientationMsg();

        imuNum++;
        magNum++;
        msgNum++;
        msgTwist++;

        if(imuMsg.header.stamp.sec==0 && imuMsg.header.stamp.nsec==0)
        {
            std::cerr<<"WARNING: IMU message " << imuNum++ << " is invalid" << std::endl;
            return(nextLine());
        } else if(magMsg.header.stamp.sec==0&& magMsg.header.stamp.nsec==0)
        {
            std::cerr<<"WARNINIG: MAG message " << magNum++ << " is invlid" << std::endl;
            return(nextLine());
        }
        return true;
    }
    else
    {
        return false;
    }
}

void UM6_reader::packImuMsg()
{
    timestampToDouble_t currentTime = (data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    imuMsg.header.stamp = stamp;
    imuMsg.header.frame_id = "imu";
    imuMsg.header.seq = imuNum;

//    tf::Quaternion q;
//    q.setRPY(data.phi, data.psi, data.theta);
//    imuMsg.orientation.x = q.x();
//    imuMsg.orientation.y = q.y();
//    imuMsg.orientation.z = q.z();
//    imuMsg.orientation.w = q.w();

//    // Column vector g
//    tf::Vector3 vec;
//    vec.setX(data.accelx*9.80665);
//    vec.setY(data.accely*9.80665);
//    vec.setZ(data.accelz);

//    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
//    tf::Matrix3x3 wf(q);

//    tf::Vector3 linaccfilt = wf*vec;
//    linaccfilt.setZ(linaccfilt.getZ()-9.80665);

//    imuMsg.angular_velocity.x = data.gyrox;
//    imuMsg.angular_velocity.y = data.gyroy;
//    imuMsg.angular_velocity.z = data.gyroz;

//    imuMsg.linear_acceleration.x =  linaccfilt.getX();
//    imuMsg.linear_acceleration.y =  -linaccfilt.getY();
//    imuMsg.linear_acceleration.z =  linaccfilt.getZ();

    imuMsg.linear_acceleration.x = (data.accelx)*9.80665;
    imuMsg.linear_acceleration.y = (data.accely)*9.80665;
    imuMsg.linear_acceleration.z = (data.accelz)*9.80665;


}

void UM6_reader::packTwistMsg()
{
//    timestampToDouble_t currentTime = double(data.timestamp);
//    ros::Time stamp(currentTime);
//    twcMsg.header.stamp = stamp;
//    twcMsg.header.frame_id = "twist";
//    twcMsg.header.seq = msgTwist;

//    twcMsg.twist.twist.angular.x = data.gyrox;
//    twcMsg.twist.twist.angular.y = data.gyroy;
//    twcMsg.twist.twist.angular.z = data.gyroz;
}

void UM6_reader::packOdometryMsg()
{
    timestampToDouble_t currentTime = (data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    odomMsg.header.stamp = stamp;
    odomMsg.header.frame_id = "odom";
    odomMsg.header.seq = imuNum;
}

void UM6_reader::packMagMsg()
{
    timestampToDouble_t currentTime = (data.timestamp)/1e6;
    ros::Time stamp(currentTime);
    magMsg.header.stamp = stamp;
    magMsg.header.frame_id = "imu";
    magMsg.header.seq = magNum;
    magMsg.magnetic_field.x = data.magx;
    magMsg.magnetic_field.y = data.magy;
    magMsg.magnetic_field.z = data.magz;
}

void UM6_reader::packAngVelMsg()
{
    double doubleTime = double(data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    angVelMsg.header.stamp = stamp;
    angVelMsg.header.frame_id = "imu";
    angVelMsg.header.seq = msgNum;
    angVelMsg.angular_velocity.x = data.gyrox;
    angVelMsg.angular_velocity.y = data.gyroy;
    angVelMsg.angular_velocity.z = data.gyroz;
}

void UM6_reader::packOrientationMsg()
{
    double doubleTime = double(data.timestamp)/1e6;
    ros::Time stamp(doubleTime);
    imuMsg.header.stamp = stamp;
    imuMsg.header.frame_id = "imu";
    imuMsg.header.seq = msgNum;
}




