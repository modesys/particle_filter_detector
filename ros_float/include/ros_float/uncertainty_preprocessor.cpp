#include "uncertainty_preprocessor.h"

uncertainty_preProcessor::uncertainty_preProcessor()
{
    node_.param("imu/angular_velocity_covariance",angular_velocity_covariance,0.1);
    node_.param("imu/orientation_covariance",orientation_covariance,0.1);
    node_.param("imu/linear_acceleration_covariance",linear_acceleration_covariance,0.1);
    node_.param("imu/frame_id", imuFrameID);

    imuPub_ = node_.advertise<sensor_msgs::Imu>("/filter/imu/data", 1);
    imuSub_ = node_.subscribe<sensor_msgs::Imu> ("/imu/data", 100, &uncertainty_preProcessor::imuCallback, this);

    node_.param("keller/depth_covariance", depth_cov);
    node_.param("keller/tempeartue_covariance", temperature_cov);
    node_.param("keller/pressure_covariance", pressure_cov);
    node_.param("keller/frame_id", kellerFrameID);

    kellerPressurePub_ = node_.advertise<sensor_msgs::FluidPressure> ("/filter/fluid_pressure/pressure", 1);
    kellerPressureSub_ = node_.subscribe<sensor_msgs::FluidPressure> ("/fluid_pressure/pressure", 100, &uncertainty_preProcessor::kellerPressureCallback, this);

    kellerTemperaturePub_ = node_.advertise<sensor_msgs::FluidPressure> ("/filter/fluid_pressure/temperature", 1);
    kellerTemperatureSub_ = node_.subscribe<sensor_msgs::FluidPressure> ("/fluid_pressure/temperature", 100, &uncertainty_preProcessor::kellerTemperatureCallback, this);

    kellerDepthPub_ = node_.advertise<sensor_msgs::FluidPressure> ("/filter/fluid_pressure/depth", 1);
    kellerDepthSub_ = node_.subscribe<sensor_msgs::FluidPressure> ("/fluid_pressure/depth", 100, &uncertainty_preProcessor::kellerDepthCallback, this);
    // Needed to transform the Fluid Pressure message into an Odometry Message
    kellerFromPressureToOdomPub_ = node_.advertise<nav_msgs::Odometry>("/filter/fluid_pressure/Odom_depth", 1);

    node_.param("mag/magnetic_covariance", magnetic_covariance, 0.1);
    node_.param("mag/magnetic_field", magnetic_field, 0.1);
    node_.param("mag/frame_id", magFrameID);

    magPub_ = node_.advertise<sensor_msgs::MagneticField> ("/filter/imu/mag", 1);
    magSub_ = node_.subscribe<sensor_msgs::MagneticField> ("/imu/mag", 100, &uncertainty_preProcessor::magCallback, this);

    node_.param("altitude/altitude_covariance", altitude, 0.1);
    node_.param("altitude/frame_id", altFrameID);

    altPub_ = node_.advertise<ros_float::altitude> ("/filter/alt/data", 1);
    altSub_ = node_.subscribe ("/alt/altitude", 100, &uncertainty_preProcessor::altCallback, this);

}


void uncertainty_preProcessor::imuCallback(const sensor_msgs::ImuConstPtr &msgIn){
    sensor_msgs::Imu msgWithCov = *msgIn;
    msgWithCov.orientation_covariance[0]=orientation_covariance;
    msgWithCov.orientation_covariance[4]=orientation_covariance;
    msgWithCov.orientation_covariance[8]=orientation_covariance;

    msgWithCov.angular_velocity_covariance[0]=angular_velocity_covariance;
    msgWithCov.angular_velocity_covariance[4]=angular_velocity_covariance;
    msgWithCov.angular_velocity_covariance[8]=angular_velocity_covariance;

    msgWithCov.linear_acceleration_covariance[0]=linear_acceleration_covariance;
    msgWithCov.linear_acceleration_covariance[4]=linear_acceleration_covariance;
    msgWithCov.linear_acceleration_covariance[8]=linear_acceleration_covariance;

    // fixes gyro bias snd it converts to radians
    msgWithCov.angular_velocity.x = ((msgWithCov.angular_velocity.x)+0.109)*3.14159/180;
    msgWithCov.angular_velocity.y = ((msgWithCov.angular_velocity.y)+2.514)*3.14159/180;
    msgWithCov.angular_velocity.z = ((msgWithCov.angular_velocity.z)+0.125)*3.14159/180;

    msgWithCov.header.frame_id = "base_link";

    imuPub_.publish(msgWithCov);
}

void uncertainty_preProcessor::kellerPressureCallback(const sensor_msgs::FluidPressureConstPtr &msgKelIn)
{
    sensor_msgs::FluidPressure msgPressure = *msgKelIn;
    msgPressure.variance = pressure_cov;
    kellerPressurePub_.publish(msgPressure);
}

void uncertainty_preProcessor::kellerTemperatureCallback(const sensor_msgs::FluidPressureConstPtr &msgKelIn)
{
    sensor_msgs::FluidPressure msgTemperature = *msgKelIn;
    msgTemperature.variance = temperature_cov;
    kellerTemperaturePub_.publish(msgTemperature);
}

void uncertainty_preProcessor::kellerDepthCallback(const sensor_msgs::FluidPressureConstPtr &msgKelIn)
{
    sensor_msgs::FluidPressure msgDepth = *msgKelIn;
    msgDepth.variance = depth_cov;
    msgDepth.header.frame_id = "base_link";
    kellerDepthPub_.publish(msgDepth);

   // Create the odometry message
    nav_msgs::Odometry msgOdom;
    msgOdom.header.stamp = msgDepth.header.stamp;
    msgOdom.header.frame_id = "map"; // <-- Double check if needed
    msgOdom.pose.pose.position.x = 0.0;
    msgOdom.pose.pose.position.y = 0.0;
    msgOdom.pose.pose.position.z = -msgDepth.fluid_pressure;
    msgOdom.pose.covariance[14] = depth_cov;
   // Publish Odometry message
    kellerFromPressureToOdomPub_.publish(msgOdom);
}

void uncertainty_preProcessor::magCallback(const sensor_msgs::MagneticFieldConstPtr &msgMagField)
{
    sensor_msgs::MagneticField msgMag = *msgMagField;
    msgMag.magnetic_field_covariance[0] = magnetic_covariance;
    msgMag.magnetic_field_covariance[4] = magnetic_covariance;
    msgMag.magnetic_field_covariance[8] = magnetic_covariance;
    msgMag.header.frame_id = "base_link";
    magPub_.publish(msgMag);
}

void uncertainty_preProcessor::altCallback(const ros_float::altitude &altMsg)
{
    ros_float::altitude msgAlt;
    msgAlt.altitude = altMsg.altitude;
    msgAlt.header.frame_id = "base_link";
    msgAlt.header.stamp = altMsg.header.stamp;
    altPub_.publish(msgAlt);
}


