#ifndef UNCERTAINTY_PREPROCESSOR_H
#define UNCERTAINTY_PREPROCESSOR_H

#define PI 3.14159265358979323846  /* pi */

#include "csv.h"
#include <stdio.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include "ros_float/altitude.h"


class uncertainty_preProcessor
{
public:
  uncertainty_preProcessor();
  void imuCallback(const sensor_msgs::ImuConstPtr& msgIn);
  void kellerPressureCallback(const sensor_msgs::FluidPressureConstPtr& msgKelIn);
  void kellerTemperatureCallback(const sensor_msgs::FluidPressureConstPtr& msgKelIn);
  void kellerDepthCallback(const sensor_msgs::FluidPressureConstPtr& msgKelIn);

  void magCallback(const sensor_msgs::MagneticFieldConstPtr& msgMagField);

  void altCallback(const ros_float::altitude& altMsg);



private:   // Publish and Subscribe the Magnetic Field too
  ros::NodeHandle node_;
  ros::Publisher imuPub_;
  ros::Subscriber imuSub_;

  ros::NodeHandle kellerNode_;

  ros::Publisher kellerPressurePub_;
  ros::Subscriber kellerPressureSub_;

  ros::Publisher kellerTemperaturePub_;
  ros::Subscriber kellerTemperatureSub_;

  ros::Publisher kellerDepthPub_;
  ros::Subscriber kellerDepthSub_;

  ros::Publisher kellerFromPressureToOdomPub_;

  ros::Publisher magPub_;
  ros::Subscriber magSub_;

  ros::Publisher altPub_;
  ros::Subscriber altSub_;

  double angular_velocity_covariance, orientation_covariance,linear_acceleration_covariance;
  double pressure_cov, temperature_cov, depth_cov;
  double magnetic_covariance, magnetic_field;
  double altitude;

  std::string kellerFrameID;
  std::string imuFrameID;
  std::string magFrameID;
  std::string altFrameID;

};

#endif // UNCERTAINTY_PREPROCESSOR_H





















