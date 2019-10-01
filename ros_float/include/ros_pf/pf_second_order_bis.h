#ifndef __NON_LINEAR_SYSTEM_MOBILE__
#define __NON_LINEAR_SYSTEM_MOBILE__

//#include <pdf/conditionalpdf.h>
//#include <pdf/gaussian.h>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
//#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/pdf/conditionalpdf.h>

#include <bfl/filter/bootstrapfilter.h>

#include "ros/ros.h"
#include <sstream>
#include <stdint.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "ros_float/altitude.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/GetMap.h>
#include <amcl/map/map.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#include <pdf/linearanalyticmeasurementmodel_gaussianuncertainty.h>
using namespace MatrixWrapper;
using namespace std;
using namespace BFL;
using namespace grid_map;

namespace BFL
{
//this class represents the nonlinear system model
class ROVModel : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    ROVModel(const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(5,2)
    {
        _additiveNoise = additiveNoise;
    }

    virtual ~ROVModel(){}

    // the following two functions must be implemented
    // this function is the function f of the discretized model: x_k+1 = f(x_k, u_k)
    //virtual bool SampleFrom (Sample<ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL);
    virtual bool SampleFrom (Sample<MatrixWrapper::ColumnVector>& one_sample, int method=DEFAULT, void * args=NULL) const;


private:
    Gaussian _additiveNoise;
    map_t* _map;

};

// this class represents the nonlinear measurement model of the USBL
class USBLModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    USBLModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(5,2){
            _measNoise = additiveNoise;
    }

    virtual ~USBLModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise; //make this a 3x3 matrix in the constructor of MyParticleFilter
};


// this class represents the nonlinear measurement model of the Keller
class KellerModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    KellerModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~KellerModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};

// this class represents the nonlinear measurement model of the camera
class CameraModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    CameraModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~CameraModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};

class AltitudeModelPdf: public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
{
public:
    AltitudeModelPdf( const Gaussian& additiveNoise): ConditionalPdf<ColumnVector,ColumnVector>(1,2)
    {
         _measNoise = additiveNoise;
    }

    virtual ~AltitudeModelPdf(){}

    // this function must be reimplemented and is the function that provides the rating of
    // particles given the measurement
    virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

private:
    Gaussian _measNoise;
};

class MyParticleFilter
{
public:
    MyParticleFilter(ros::NodeHandle &n);
    ~MyParticleFilter()
    {
        delete filter;
        delete sys_model;
        delete usbl_meas_model;
        delete keller_meas_model;
        delete sys_pdf;
        delete keller_meas_pdf;
        delete camera_meas_model;
        delete camera_meas_pdf;
        delete alt_meas_pdf;
        delete altitude_meas_model;
    }

    double roll, pitch, yaw;

    // create grid map
    grid_map::GridMap map;

    BootstrapFilter<ColumnVector,ColumnVector>* filter;

    ros::NodeHandle _n;
    ros::Subscriber usbl_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber kell_sub;
    sensor_msgs::Imu imuMsg;

    ros::Publisher particle_pub;
    ros::Publisher pose_pub;

    ros::Publisher pose_withcovariance_pub;

    ros::Publisher acc_pub;

    ros::Timer particleTimer;

    // From the camera Odometry
    ros::Subscriber odometry_sub;
    nav_msgs::Odometry odom;

    ros::Subscriber alt_pub;

    ///
    /// \brief lastAltitudeReceived: save the last altitude error message received
    ///
    ros_float::altitude lastAltitudeReceived;

    ///
    /// \brief altErrorPublisher: it published the error on the latimeter
    ///
    ros::Publisher altErrorPublisher;


    private:
        double lat, lon, usbl_used_inresampling;

        SystemModel<ColumnVector>* sys_model;
        ROVModel* sys_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* usbl_meas_model;
        USBLModelPdf* usbl_meas_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* keller_meas_model;
        KellerModelPdf* keller_meas_pdf;

        MeasurementModel<ColumnVector,ColumnVector>* camera_meas_model;
        CameraModelPdf* camera_meas_pdf;

        MeasurementModel<ColumnVector, ColumnVector>* altitude_meas_model;
        AltitudeModelPdf* alt_meas_pdf;


        void camCb(const nav_msgs::OdometryConstPtr& msg)
        {
            double linearVelocity_X = msg->twist.twist.linear.x;
            double linearVelocity_Y = msg->twist.twist.linear.y;
            double linearVelocity_Z = msg->twist.twist.linear.z;

            double twistCovariance_X = msg->twist.covariance[0];
            double twistCovariance_Y = msg->twist.covariance[7];
            double twistCovariance_Z = 0.0;

            ColumnVector camMeasurement(6);

            camMeasurement(1) = linearVelocity_X/3.0;
            camMeasurement(2) = linearVelocity_Y/3.0;
            camMeasurement(3) = linearVelocity_Z/3.0;
            camMeasurement(4) = twistCovariance_X;
            camMeasurement(5) = twistCovariance_Y;
            camMeasurement(6) = twistCovariance_Z;

            std::cout<<std::endl<<" CAM      "<< camMeasurement(1) << " "<< camMeasurement(2) << "  "<< camMeasurement(3)<<std::endl;

            filter->Update(camera_meas_model, camMeasurement);

        }

        ///
        /// \brief imuCb: This is the Inertial MEasurment Unit callback. And the Update function happens
        /// \brief everytime an IMU message is received. However we had to introduce:
        /// \brief 1) ColumnVector for gravity g(3) that has 3 components
        /// \brief 2) A Rotation Mtrix rot(3,3) so that we could proceed with
        /// \brief multiply the linear acceleration x Rotation Mtrix rot(3,3) and take out g(3)
        /// \param msg
        ///
        void imuCb(const sensor_msgs::Imu::ConstPtr& imuMsg)
        {
            tf::Quaternion q(imuMsg->orientation.x, imuMsg->orientation.y,
                             imuMsg->orientation.z, imuMsg->orientation.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            tf::Matrix3x3 wf(q);

            ColumnVector lin_acc(3);
            lin_acc(1)= imuMsg->linear_acceleration.x;
            lin_acc(2)= imuMsg->linear_acceleration.y;
            lin_acc(3)= imuMsg->linear_acceleration.z;

            ColumnVector lin_acc_filtered(3);
            ColumnVector g(3);
            g(1) = 0;
            g(2) = 0;
            g(3) = 9.8;

            MatrixWrapper::Matrix rot(3,3);
            rot(1,1) = wf[0][0];
            rot(1,2) = wf[0][1];
            rot(1,3) = wf[0][2];
            rot(2,1) = wf[1][0];
            rot(2,2) = wf[1][1];
            rot(2,3) = wf[1][2];
            rot(3,1) = wf[2][0];
            rot(3,2) = wf[2][1];
            rot(3,3) = wf[2][2];

            lin_acc_filtered = rot*lin_acc;
            lin_acc_filtered = lin_acc_filtered - g;

            ColumnVector lin_acc_filtered_reduced(5);
            lin_acc_filtered_reduced(1) = lin_acc_filtered(1);
            lin_acc_filtered_reduced(2) = lin_acc_filtered(2);

            lin_acc_filtered_reduced(3) = usbl_used_inresampling;
            lin_acc_filtered_reduced(4) = lat;
            lin_acc_filtered_reduced(5) = lon;
            usbl_used_inresampling = 1.0;

            std::cout<<".";
            filter->Update(sys_model, lin_acc_filtered_reduced);
        }

        ///
        /// \brief altCb: This is the Altitide callback. The Update happens everytime
        /// \brief an altitude message is received
        /// \param altitudeMsg: This is the ALT message
        ///
        void altCb(const ros_float::altitude& altitudeMsg)
        {
            ColumnVector alt(1);
            alt(1) = altitudeMsg.altitude;
            // saving the last altitude message received
            lastAltitudeReceived = altitudeMsg;

            std::cout<<std::endl<<" ALT      "<< alt(1)<<std::endl;

            filter->Update(altitude_meas_model, alt);
        }

        ///
        /// \brief kellCb: This is the depth callback. The Update happens eveytime
        /// \brief a depth message is received
        /// \param depthMsg: This is the DEPTH message
        ///
        void kellCb(const sensor_msgs::FluidPressure::ConstPtr& depthMsg)
        {
            ColumnVector press(1);
            press(1) = depthMsg->fluid_pressure;
            std::cout<<"K "<< press(1)<<" ";
            filter->Update(keller_meas_model, press);
        }

        ///
        /// \brief usblCb: This is the underwater positionin system callbak
        /// \brief remeber that USBL provide XYZ underwater but only X-Y are used
        /// \brief as the depth is too noisy and the Depth sensor Keller was used for this
        /// \param usblMsg: This is the USBL message
        ///
        void usblCb(const nav_msgs::Odometry::ConstPtr& usblMsg)
        {
            lat = usblMsg->pose.pose.position.x;
            lon = usblMsg->pose.pose.position.y;
            usbl_used_inresampling = 0.0;
            double alt = usblMsg->pose.pose.position.z;

            ColumnVector measurement(2);
            measurement(1) = lat;
            measurement(2) = lon;
            measurement(3) = alt;
            measurement(4) = roll;
            measurement(5) = pitch;
            measurement(6) = yaw;

            std::cout<<std::endl<<" USBL      "<< measurement(1) << " "<< measurement(2) << "  "<< measurement(3)<<std::endl;

            filter->Update(usbl_meas_model, measurement);
        }

        void timerCallback(const ros::TimerEvent&)
        {
            publishParticles();
            publishPose();
        }

        // Publishing particles for the whole system
        void publishParticles()
        {
            geometry_msgs::PoseArray particleCloudMsg;
            particleCloudMsg.header.stamp = ros::Time::now();
            particleCloudMsg.header.frame_id = "map";

            std::vector<WeightedSample<ColumnVector>>::iterator sample_it;
            std::vector<WeightedSample<ColumnVector>> samples;

            samples = filter->PostGet()->ListOfSamplesGet();

            for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
            {
                geometry_msgs::Pose pose;
                ColumnVector sample = (*sample_it).ValueGet();

                pose.position.x = -sample(1);
                pose.position.y = -sample(2);
                pose.position.z = -sample(3);

                particleCloudMsg.poses.insert(particleCloudMsg.poses.begin(), pose);
            }
            particle_pub.publish(particleCloudMsg);
        }

        void publishPose(); // publish of the estimate of the state


    };

} // End namespace BFL

#endif //
